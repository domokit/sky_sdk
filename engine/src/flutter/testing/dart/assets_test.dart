// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:convert';
import 'dart:typed_data';
import 'dart:ui';

import 'package:test/test.dart';

void main() {
  test('Loading an asset that does not exist returns null', () async {
    Object? error;
    try {
      await ImmutableBuffer.fromAsset('ThisDoesNotExist');
    } catch (err) {
      error = err;
    }
    expect(error, isNotNull);
    expect(error is Exception, true);
  });

  test('Loading a file that does not exist returns null', () async {
    Object? error;
    try {
      await ImmutableBuffer.fromFilePath('ThisDoesNotExist');
    } catch (err) {
      error = err;
    }
    expect(error, isNotNull);
    expect(error is Exception, true);
  });

  test('returns the bytes of a bundled asset', () async {
    final ImmutableBuffer buffer = await ImmutableBuffer.fromAsset('DashInNooglerHat.jpg');

    expect(buffer.length == 354679, true);
  });

  test('returns the bytes of a file', () async {
    final ImmutableBuffer buffer = await ImmutableBuffer.fromFilePath(
      'flutter/lib/ui/fixtures/DashInNooglerHat.jpg',
    );

    expect(buffer.length == 354679, true);
  });

  test('Can load an asset with a space in the key', () async {
    // This assets actual path is "fixtures/DashInNooglerHat%20WithSpace.jpg"
    final ImmutableBuffer buffer = await ImmutableBuffer.fromAsset(
      'DashInNooglerHat WithSpace.jpg',
    );

    expect(buffer.length == 354679, true);
  });

  test('can dispose immutable buffer', () async {
    final ImmutableBuffer buffer = await ImmutableBuffer.fromAsset('DashInNooglerHat.jpg');

    buffer.dispose();
  });

  test('Tester can disable loading fonts from an asset bundle', () async {
    final List<int> ahemImage = await _createPictureFromFont('Ahem');
    // Font that is bundled in the asset directory of the test runner.
    final List<int> bundledFontImage = await _createPictureFromFont('Roboto');
    // Bundling fonts is disabled, so the font selected in both cases should be ahem.
    // Therefore each buffer will contain identical contents.
    expect(ahemImage, equals(bundledFontImage));
  });

  test('Tester can still load through dart:ui', () async {
    /// Manually load font asset through dart.
    final Uint8List encoded = utf8.encode(Uri(path: Uri.encodeFull('Roboto-Medium.ttf')).path);
    final Completer<Uint8List> result = Completer<Uint8List>();
    PlatformDispatcher.instance.sendPlatformMessage('flutter/assets', encoded.buffer.asByteData(), (
      ByteData? data,
    ) {
      result.complete(data!.buffer.asUint8List());
    });

    await loadFontFromList(await result.future, fontFamily: 'Roboto2');

    final List<int> ahemImage = await _createPictureFromFont('Ahem');
    // Font that is bundled in the asset directory of the test runner.
    final List<int> bundledFontImage = await _createPictureFromFont('Roboto2');
    // Bundling fonts is disabled, so the font selected in both cases should be ahem.
    // Therefore each buffer will contain identical contents.
    expect(ahemImage, isNot(bundledFontImage));
  });
}

Future<List<int>> _createPictureFromFont(String fontFamily) async {
  final ParagraphBuilder builder = ParagraphBuilder(
    ParagraphStyle(
      fontFamily: fontFamily,
      fontStyle: FontStyle.normal,
      fontWeight: FontWeight.normal,
      fontSize: 20,
    ),
  );
  builder.addText('Test');
  final Paragraph paragraph = builder.build();
  paragraph.layout(const ParagraphConstraints(width: 20 * 5.0));

  final PictureRecorder recorder = PictureRecorder();
  final Canvas canvas = Canvas(recorder);
  canvas.drawParagraph(paragraph, Offset.zero);

  final Picture picture = recorder.endRecording();
  final Image image = await picture.toImage(100, 100);
  final ByteData? data = await image.toByteData();
  return data!.buffer.asUint8List().toList();
}
