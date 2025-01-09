// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:js_interop';

import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' as ui;

/// Signature of the callback that receives progress updates as image chunks are
/// loaded.
typedef ImageCodecChunkCallback = void Function(int cumulativeBytesLoaded, int expectedTotalBytes);

/// Creates a [ui.Codec] for the image located at [uri].
///
/// The [chunkCallback] is called with progress updates as image chunks are
/// loaded.
Future<ui.Codec> createImageCodecFromUrl(Uri uri, {ImageCodecChunkCallback? chunkCallback}) {
  return renderer.instantiateImageCodecFromUrl(uri, chunkCallback: chunkCallback);
}

/// Creates a [ui.Image] from an ImageBitmap object.
///
/// The contents of the ImageBitmap must have a premultiplied alpha.
/// The engine will take ownership of the ImageBitmap object and consume its
/// contents.
///
/// See https://developer.mozilla.org/en-US/docs/Web/API/ImageBitmap
FutureOr<ui.Image> createImageFromImageBitmap(JSAny imageSource) {
  if (!domInstanceOfString(imageSource, 'ImageBitmap')) {
    throw ArgumentError('Image source $imageSource is not an ImageBitmap.', 'imageSource');
  }
  return renderer.createImageFromImageBitmap(imageSource as DomImageBitmap);
}

/// Creates a [ui.Image] from a valid texture source (for example
/// HTMLImageElement | HTMLVideoElement | HTMLCanvasElement).
///
/// By default, [transferOwnership] specifies that the ownership of the texture
/// will be not be transferred to the renderer, and a copy of the texture source
/// will be made. If this is not desired, the ownership of the object can be
/// transferred to the renderer and the engine will take ownership of the
/// texture source and consume its contents.
FutureOr<ui.Image> createImageFromTextureSource(
  JSAny object, {
  required int width,
  required int height,
  bool transferOwnership = false,
}) {
  return renderer.createImageFromTextureSource(
    object,
    width: width,
    height: height,
    transferOwnership: transferOwnership,
  );
}
