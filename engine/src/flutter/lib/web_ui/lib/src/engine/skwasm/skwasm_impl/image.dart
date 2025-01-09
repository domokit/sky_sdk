// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ffi';
import 'dart:js_interop';
import 'dart:typed_data';

import 'package:ui/src/engine.dart';
import 'package:ui/src/engine/skwasm/skwasm_impl.dart';
import 'package:ui/ui.dart' as ui;

class SkwasmImage extends SkwasmObjectWrapper<RawImage> implements ui.Image {
  SkwasmImage(ImageHandle handle) : super(handle, _registry) {
    ui.Image.onCreate?.call(this);
  }

  factory SkwasmImage.fromPixels(
    Uint8List pixels,
    int width,
    int height,
    ui.PixelFormat format, {
    int? rowBytes,
  }) {
    final SkDataHandle dataHandle = skDataCreate(pixels.length);
    final Pointer<Uint8> dataPointer = skDataGetPointer(dataHandle).cast<Uint8>();
    for (int i = 0; i < pixels.length; i++) {
      dataPointer[i] = pixels[i];
    }
    final ImageHandle imageHandle = imageCreateFromPixels(
      dataHandle,
      width,
      height,
      format.index,
      rowBytes ?? 4 * width,
    );
    skDataDispose(dataHandle);
    return SkwasmImage(imageHandle);
  }

  static final SkwasmFinalizationRegistry<RawImage> _registry =
      SkwasmFinalizationRegistry<RawImage>(imageDispose);

  @override
  void dispose() {
    super.dispose();
    ui.Image.onDispose?.call(this);
  }

  @override
  int get width => imageGetWidth(handle);

  @override
  int get height => imageGetHeight(handle);

  @override
  Future<ByteData?> toByteData({ui.ImageByteFormat format = ui.ImageByteFormat.rawRgba}) async {
    if (format == ui.ImageByteFormat.png) {
      final ui.PictureRecorder recorder = ui.PictureRecorder();
      final ui.Canvas canvas = ui.Canvas(recorder);
      canvas.drawImage(this, ui.Offset.zero, ui.Paint());
      final DomImageBitmap bitmap =
          (await (renderer as SkwasmRenderer).surface.renderPictures(<SkwasmPicture>[
            recorder.endRecording() as SkwasmPicture,
          ])).imageBitmaps.single;
      final DomOffscreenCanvas offscreenCanvas = createDomOffscreenCanvas(
        bitmap.width.toDartInt,
        bitmap.height.toDartInt,
      );
      final DomCanvasRenderingContextBitmapRenderer context =
          offscreenCanvas.getContext('bitmaprenderer')! as DomCanvasRenderingContextBitmapRenderer;
      context.transferFromImageBitmap(bitmap);
      final DomBlob blob = await offscreenCanvas.convertToBlob();
      final JSArrayBuffer arrayBuffer = (await blob.arrayBuffer().toDart)! as JSArrayBuffer;

      // Zero out the contents of the canvas so that resources can be reclaimed
      // by the browser.
      context.transferFromImageBitmap(null);
      return ByteData.view(arrayBuffer.toDart);
    } else {
      return (renderer as SkwasmRenderer).surface.rasterizeImage(this, format);
    }
  }

  @override
  ui.ColorSpace get colorSpace => ui.ColorSpace.sRGB;

  @override
  SkwasmImage clone() {
    imageRef(handle);
    return SkwasmImage(handle);
  }

  @override
  bool isCloneOf(ui.Image other) => other is SkwasmImage && handle == other.handle;

  @override
  List<StackTrace>? debugGetOpenHandleStackTraces() => null;

  @override
  String toString() => '[$width\u00D7$height]';
}
