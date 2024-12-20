// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui' as ui;

import 'image_stream.dart';

/// An [ImageInfo] object indicating that the image can only be displayed in
/// an <img> element, and no [dart:ui.Image] can be created for it.
///
/// This occurs on the web when the image resource is from a different origin
/// and is not configured for CORS. Since the image bytes cannot be directly
/// fetched, [ui.Image]s cannot be created from it. However, the image can
/// still be displayed if an <img> element is used.
class WebImageInfo implements ImageInfo {
  @override
  ImageInfo clone() => _unsupported();

  @override
  String? get debugLabel => _unsupported();

  @override
  void dispose() => _unsupported();

  @override
  ui.Image get image => _unsupported();

  @override
  bool isCloneOf(ImageInfo other) => _unsupported();

  @override
  double get scale => _unsupported();

  @override
  int get sizeBytes => _unsupported();

  Never _unsupported() =>
      throw UnsupportedError('WebImageInfo should never be instantiated in a non-web context.');
}
