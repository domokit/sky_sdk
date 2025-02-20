// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' as ui;
import 'package:ui/ui_web/src/ui_web.dart' as ui_web;

abstract class HtmlImageElementCodec implements ui.Codec {
  HtmlImageElementCodec(this.src, {this.chunkCallback, this.debugSource});

  final String src;
  final ui_web.ImageCodecChunkCallback? chunkCallback;
  final String? debugSource;

  @override
  int get frameCount => 1;

  @override
  int get repetitionCount => 0;

  /// The Image() element backing this codec.
  DomHTMLImageElement? imgElement;

  /// A Future which completes when the Image element backing this codec has
  /// been loaded and decoded.
  Future<void>? decodeFuture;

  Future<void> decode() async {
    if (decodeFuture != null) {
      return decodeFuture;
    }
    final Completer<void> completer = Completer<void>();
    decodeFuture = completer.future;
    // Currently there is no way to watch decode progress, so
    // we add 0/100 , 100/100 progress callbacks to enable loading progress
    // builders to create UI.
    chunkCallback?.call(0, 100);
    imgElement = createDomHTMLImageElement();
    imgElement!.crossOrigin = 'anonymous';
    imgElement!
      ..decoding = 'async'
      ..src = src;

    // Ignoring the returned future on purpose because we're communicating
    // through the `completer`.
    // ignore: unawaited_futures
    imgElement!
        .decode()
        .then((dynamic _) {
          chunkCallback?.call(100, 100);
          completer.complete();
        })
        .catchError((dynamic e) {
          completer.completeError(e.toString());
        });
    return completer.future;
  }

  @override
  Future<ui.FrameInfo> getNextFrame() async {
    await decode();
    int naturalWidth = imgElement!.naturalWidth.toInt();
    int naturalHeight = imgElement!.naturalHeight.toInt();
    // Workaround for https://bugzilla.mozilla.org/show_bug.cgi?id=700533.
    if (naturalWidth == 0 &&
        naturalHeight == 0 &&
        ui_web.browser.browserEngine == ui_web.BrowserEngine.firefox) {
      const int kDefaultImageSizeFallback = 300;
      naturalWidth = kDefaultImageSizeFallback;
      naturalHeight = kDefaultImageSizeFallback;
    }
    final ui.Image image = createImageFromHTMLImageElement(
      imgElement!,
      naturalWidth,
      naturalHeight,
    );
    return SingleFrameInfo(image);
  }

  /// Creates a [ui.Image] from an [HTMLImageElement] that has been loaded.
  ui.Image createImageFromHTMLImageElement(
    DomHTMLImageElement image,
    int naturalWidth,
    int naturalHeight,
  );

  @override
  void dispose() {}
}

abstract class HtmlBlobCodec extends HtmlImageElementCodec {
  HtmlBlobCodec(this.blob, {super.chunkCallback})
    : super(domWindow.URL.createObjectURL(blob), debugSource: 'encoded image bytes');

  final DomBlob blob;

  @override
  void dispose() {
    domWindow.URL.revokeObjectURL(src);
  }
}

class SingleFrameInfo implements ui.FrameInfo {
  SingleFrameInfo(this.image);

  @override
  Duration get duration => Duration.zero;

  @override
  final ui.Image image;
}
