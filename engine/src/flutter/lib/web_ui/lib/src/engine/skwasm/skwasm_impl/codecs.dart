// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:js_interop';

import 'package:ui/src/engine.dart';
import 'package:ui/src/engine/skwasm/skwasm_impl.dart';
import 'package:ui/ui.dart' as ui;

class SkwasmImageDecoder extends BrowserImageDecoder {
  SkwasmImageDecoder({
    required super.contentType,
    required super.dataSource,
    required super.debugSource,
  });

  @override
  ui.Image generateImageFromVideoFrame(VideoFrame frame) {
    final int width = frame.displayWidth.toInt();
    final int height = frame.displayHeight.toInt();
    final SkwasmSurface surface = (renderer as SkwasmRenderer).surface;
    return SkwasmImage(
      imageCreateFromTextureSource(frame as JSObject, width, height, surface.handle),
    );
  }
}
