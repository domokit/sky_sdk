// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:typed_data';
import 'dart:ui' as ui;

import 'package:flutter/painting.dart';
import 'package:flutter_test/flutter_test.dart';

import '../image_data.dart';
import 'painting_utils.dart';
import 'dart:io';

import 'dart:async';
import 'dart:js_interop';
import 'dart:typed_data';



import 'package:path/path.dart' as path;

void main() {
  final PaintingBindingSpy binding = PaintingBindingSpy();
  test('Image decoder control test', () async {
    expect(binding.instantiateImageCodecCalledCount, 0);
    final ui.Image image = await decodeImageFromList(Uint8List.fromList(kTransparentImage));
    expect(image, isNotNull);
    expect(image.width, 1);
    expect(image.height, 1);
    expect(binding.instantiateImageCodecCalledCount, 1);
  });

  test('ResizingCodec constructor invokes onCreate once', () async {
    int onCreateInvokedCount = 0;

    ui.Codec? createdCodec;
    ui.Codec.onCreate = (ui.Codec codec) {
      onCreateInvokedCount++;
      createdCodec = codec;
    };

    final ui.Codec codec1 = await _createCodec();

    expect(onCreateInvokedCount, 1);
    expect(createdCodec, codec1);

    final ui.Codec codec2 = await _createCodec();

    expect(onCreateInvokedCount, 2);
    expect(createdCodec, codec2);

    ui.Codec.onCreate = null;
  });

  test('dispose() invokes onDispose once', () async {
    int onDisposeInvokedCount = 0;
    ui.Codec? disposedCodec;
    ui.Codec.onDispose = (ui.Codec codec) {
      onDisposeInvokedCount++;
      disposedCodec = codec;
    };

    final ui.Codec codec1 =
        await _createCodec()
          ..dispose();

    expect(onDisposeInvokedCount, 1);
    expect(disposedCodec, codec1);

    final ui.Codec codec2 =
        await _createCodec()
          ..dispose();

    expect(onDisposeInvokedCount, 2);
    expect(disposedCodec, codec2);

    ui.Codec.onDispose = null;
  });
}

Future<ui.Codec> _createCodec() async {
  Uint8List data = await _getSkiaResource('test640x479.gif').readAsBytes();
  final ui.ImmutableBuffer buffer = await ui.ImmutableBuffer.fromUint8List(data);
  final ui.Codec codec = await PaintingBinding.instance.instantiateImageCodecWithSize(buffer);

  assert

  return codec;
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
