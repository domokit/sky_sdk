// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:typed_data';

import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' as ui;

class HtmlRendererImageCodec extends HtmlImageElementCodec {
  HtmlRendererImageCodec(super.src, {super.chunkCallback});

  @override
  ui.Image createImageFromHTMLImageElement(
    DomHTMLImageElement image,
    int naturalWidth,
    int naturalHeight,
  ) {
    return HtmlImage(image, naturalWidth, naturalHeight);
  }
}

class HtmlRendererBlobCodec extends HtmlBlobCodec {
  HtmlRendererBlobCodec(super.blob);

  @override
  ui.Image createImageFromHTMLImageElement(
    DomHTMLImageElement image,
    int naturalWidth,
    int naturalHeight,
  ) {
    return HtmlImage(image, naturalWidth, naturalHeight);
  }
}

class HtmlImage implements ui.Image {
  HtmlImage(this.imgElement, this.width, this.height) {
    ui.Image.onCreate?.call(this);
  }

  final DomHTMLImageElement imgElement;
  bool _didClone = false;

  bool _disposed = false;
  @override
  void dispose() {
    ui.Image.onDispose?.call(this);
    // Do nothing. The codec that owns this image should take care of
    // releasing the object url.
    assert(() {
      _disposed = true;
      return true;
    }());
  }

  @override
  bool get debugDisposed {
    bool? result;
    assert(() {
      result = _disposed;
      return true;
    }());

    if (result != null) {
      return result!;
    }

    throw StateError('Image.debugDisposed is only available when asserts are enabled.');
  }

  @override
  ui.Image clone() => this;

  @override
  bool isCloneOf(ui.Image other) => other == this;

  @override
  List<StackTrace>? debugGetOpenHandleStackTraces() => null;

  @override
  final int width;

  @override
  final int height;

  @override
  Future<ByteData?> toByteData({ui.ImageByteFormat format = ui.ImageByteFormat.rawRgba}) {
    switch (format) {
      // TODO(ColdPaleLight): https://github.com/flutter/flutter/issues/89128
      // The format rawRgba always returns straight rather than premul currently.
      case ui.ImageByteFormat.rawRgba:
      case ui.ImageByteFormat.rawStraightRgba:
        final DomCanvasElement canvas =
            createDomCanvasElement()
              ..width = width.toDouble()
              ..height = height.toDouble();
        final DomCanvasRenderingContext2D ctx = canvas.context2D;
        ctx.drawImage(imgElement, 0, 0);
        final DomImageData imageData = ctx.getImageData(0, 0, width, height);
        // Resize the canvas to 0x0 to cause the browser to reclaim its memory
        // eagerly.
        canvas.width = 0;
        canvas.height = 0;
        return Future<ByteData?>.value(imageData.data.buffer.asByteData());
      default:
        if (imgElement.src?.startsWith('data:') ?? false) {
          final UriData data = UriData.fromUri(Uri.parse(imgElement.src!));
          return Future<ByteData?>.value(data.contentAsBytes().buffer.asByteData());
        } else {
          return Future<ByteData?>.value();
        }
    }
  }

  @override
  ui.ColorSpace get colorSpace => ui.ColorSpace.sRGB;

  DomHTMLImageElement cloneImageElement() {
    if (!_didClone) {
      _didClone = true;
      imgElement.style.position = 'absolute';
    }
    return imgElement.cloneNode(true) as DomHTMLImageElement;
  }

  @override
  String toString() => '[$width\u00D7$height]';
}
