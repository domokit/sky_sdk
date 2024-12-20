// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui';

import '../web.dart' as web;
import 'image_stream.dart';

/// An [ImageInfo] object indicating that the image can only be displayed in
/// an <img> element, and no [dart:ui.Image] can be created for it.
///
/// This occurs on the web when the image resource is from a different origin
/// and is not configured for CORS. Since the image bytes cannot be directly
/// fetched, [ui.Image]s cannot be created from it. However, the image can
/// still be displayed if an <img> element is used.
class WebImageInfo implements ImageInfo {
  /// Creates a new [WebImageInfo] from a given <img> element.
  WebImageInfo(this.htmlImage, {this.debugLabel});

  /// The <img> element used to display this image. This <img> element has
  /// already been decoded, so size information can be retrieved from it.
  final web.HTMLImageElement htmlImage;

  @override
  final String? debugLabel;

  @override
  WebImageInfo clone() {
    // There is no need to actually clone the <img> element here. We create
    // another reference to the <img> element and let the browser garbage
    // collect it when there are no more live references.
    return WebImageInfo(htmlImage, debugLabel: debugLabel);
  }

  @override
  void dispose() {
    // There is nothing to do here. There is no way to delete an element
    // directly, the most we can do is remove it from the DOM. But the <img>
    // element here is never even added to the DOM. The browser will
    // automatically garbage collect the element when there are no longer any
    // live references to it.
  }

  @override
  Image get image =>
      throw UnsupportedError(
        'Could not create image data for this image because access to it is '
        'restricted by the Same-Origin Policy.\n'
        'See https://developer.mozilla.org/en-US/docs/Web/Security/Same-origin_policy',
      );

  @override
  bool isCloneOf(ImageInfo other) {
    if (other is! WebImageInfo) {
      return false;
    }

    // It is a clone if it points to the same <img> element.
    return other.htmlImage == htmlImage && other.debugLabel == debugLabel;
  }

  @override
  double get scale => 1.0;

  @override
  int get sizeBytes => (4 * htmlImage.naturalWidth * htmlImage.naturalHeight).toInt();
}
