// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:typed_data';

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine/canvaskit/image.dart';
import 'package:ui/src/engine/dom.dart';
import 'package:ui/src/engine/html/image.dart';
import 'package:ui/src/engine/html_image_element_codec.dart';
import 'package:ui/ui.dart' as ui;
import 'package:ui/ui_web/src/ui_web.dart' as ui_web;

import '../../common/test_initialization.dart';
import '../../ui/utils.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

Future<void> testMain() async {
  setUpUnitTests();
  group('$HtmlImageElementCodec', () {
    test('supports raw images - RGBA8888', () async {
      final Completer<ui.Image> completer = Completer<ui.Image>();
      const int width = 200;
      const int height = 300;
      final Uint32List list = Uint32List(width * height);
      for (int index = 0; index < list.length; index += 1) {
        list[index] = 0xFF0000FF;
      }
      ui.decodeImageFromPixels(
        list.buffer.asUint8List(),
        width,
        height,
        ui.PixelFormat.rgba8888,
        (ui.Image image) => completer.complete(image),
      );
      final ui.Image image = await completer.future;
      expect(image.width, width);
      expect(image.height, height);
    });
    test('supports raw images - BGRA8888', () async {
      final Completer<ui.Image> completer = Completer<ui.Image>();
      const int width = 200;
      const int height = 300;
      final Uint32List list = Uint32List(width * height);
      for (int index = 0; index < list.length; index += 1) {
        list[index] = 0xFF0000FF;
      }
      ui.decodeImageFromPixels(
        list.buffer.asUint8List(),
        width,
        height,
        ui.PixelFormat.bgra8888,
        (ui.Image image) => completer.complete(image),
      );
      final ui.Image image = await completer.future;
      expect(image.width, width);
      expect(image.height, height);
    });
    test('loads sample image', () async {
      final HtmlImageElementCodec codec = createImageElementCodec('sample_image1.png');
      final ui.FrameInfo frameInfo = await codec.getNextFrame();

      expect(codec.imgElement, isNotNull);
      expect(codec.imgElement!.src, contains('sample_image1.png'));
      expect(codec.imgElement!.crossOrigin, isHtml ? isNull : 'anonymous');
      expect(codec.imgElement!.decoding, 'async');

      expect(frameInfo.image, isNotNull);
      expect(frameInfo.image.width, 100);
      expect(frameInfo.image.toString(), '[100×100]');
    });
    test('dispose image image', () async {
      final HtmlImageElementCodec codec = createImageElementCodec('sample_image1.png');
      final ui.FrameInfo frameInfo = await codec.getNextFrame();
      expect(frameInfo.image, isNotNull);
      expect(frameInfo.image.debugDisposed, isFalse);
      frameInfo.image.dispose();
      expect(frameInfo.image.debugDisposed, isTrue);
    });
    test('provides image loading progress', () async {
      final StringBuffer buffer = StringBuffer();
      final HtmlImageElementCodec codec = createImageElementCodec(
        'sample_image1.png',
        chunkCallback: (int loaded, int total) {
          buffer.write('$loaded/$total,');
        },
      );
      await codec.getNextFrame();
      expect(buffer.toString(), '0/100,100/100,');
    });

    /// Regression test for Firefox
    /// https://github.com/flutter/flutter/issues/66412
    test('Returns nonzero natural width/height', () async {
      final HtmlImageElementCodec codec = createImageElementCodec(
        'data:image/svg+xml;base64,PHN2ZyByb2xlPSJpbWciIHZpZXdCb3g9I'
        'jAgMCAyNCAyNCIgeG1sbnM9Imh0dHA6Ly93d3cudzMub3JnLzIwMDAvc3ZnIj48dG'
        'l0bGU+QWJzdHJhY3QgaWNvbjwvdGl0bGU+PHBhdGggZD0iTTEyIDBjOS42MDEgMCAx'
        'MiAyLjM5OSAxMiAxMiAwIDkuNjAxLTIuMzk5IDEyLTEyIDEyLTkuNjAxIDAtMTItMi'
        '4zOTktMTItMTJDMCAyLjM5OSAyLjM5OSAwIDEyIDB6bS0xLjk2OSAxOC41NjRjMi41'
        'MjQuMDAzIDQuNjA0LTIuMDcgNC42MDktNC41OTUgMC0yLjUyMS0yLjA3NC00LjU5NS'
        '00LjU5NS00LjU5NVM1LjQ1IDExLjQ0OSA1LjQ1IDEzLjk2OWMwIDIuNTE2IDIuMDY1'
        'IDQuNTg4IDQuNTgxIDQuNTk1em04LjM0NC0uMTg5VjUuNjI1SDUuNjI1djIuMjQ3aD'
        'EwLjQ5OHYxMC41MDNoMi4yNTJ6bS04LjM0NC02Ljc0OGEyLjM0MyAyLjM0MyAwIDEx'
        'LS4wMDIgNC42ODYgMi4zNDMgMi4zNDMgMCAwMS4wMDItNC42ODZ6Ii8+PC9zdmc+',
      );
      final ui.FrameInfo frameInfo = await codec.getNextFrame();
      expect(frameInfo.image.width, isNot(0));
    });
  }, skip: isSkwasm);

  group('ImageCodecUrl', () {
    test('loads sample image from web', () async {
      final Uri uri = Uri.base.resolve('sample_image1.png');
      final HtmlImageElementCodec codec =
          await ui_web.createImageCodecFromUrl(uri) as HtmlImageElementCodec;
      final ui.FrameInfo frameInfo = await codec.getNextFrame();

      expect(codec.imgElement, isNotNull);
      expect(codec.imgElement!.src, contains('sample_image1.png'));
      expect(codec.imgElement!.crossOrigin, isHtml ? isNull : 'anonymous');
      expect(codec.imgElement!.decoding, 'async');

      expect(frameInfo.image, isNotNull);
      expect(frameInfo.image.width, 100);
    });
    test('provides image loading progress from web', () async {
      final Uri uri = Uri.base.resolve('sample_image1.png');
      final StringBuffer buffer = StringBuffer();
      final HtmlImageElementCodec codec =
          await ui_web.createImageCodecFromUrl(
                uri,
                chunkCallback: (int loaded, int total) {
                  buffer.write('$loaded/$total,');
                },
              )
              as HtmlImageElementCodec;
      await codec.getNextFrame();
      expect(buffer.toString(), '0/100,100/100,');
    });
  }, skip: isSkwasm);
}

HtmlImageElementCodec createImageElementCodec(
  String src, {
  ui_web.ImageCodecChunkCallback? chunkCallback,
}) {
  return isHtml
      ? HtmlRendererImageCodec(src, chunkCallback: chunkCallback)
      : CkImageElementCodec(src, chunkCallback: chunkCallback);
}
