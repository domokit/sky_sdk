// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import '../painting/_web_image_info_io.dart';
import 'basic.dart';
import 'framework.dart';

/// A [Widget] that displays an image that is backed by an <img> element.
class RawWebImage extends StatelessWidget {
  /// Creates a [RawWebImage].
  RawWebImage({
    super.key,
    required this.image,
    this.debugImageLabel,
    this.width,
    this.height,
    this.fit,
    this.alignment = Alignment.center,
    this.matchTextDirection = false,
  }) {
    throw UnsupportedError('Cannot create a $RawWebImage when not running on the web');
  }

  /// The underlying `<img>` element to be displayed.
  final WebImageInfo image;

  /// A debug label explaining the image.
  final String? debugImageLabel;

  /// The requested width for this widget.
  final double? width;

  /// The requested height for this widget.
  final double? height;

  /// How the `<img>` should be inscribed in the box constraining it.
  final BoxFit? fit;

  /// How the image should be aligned in the box constraining it.
  final AlignmentGeometry alignment;

  /// Whether or not the alignment of the image should match the text direction.
  final bool matchTextDirection;

  @override
  Widget build(BuildContext context) {
    throw UnsupportedError(
      'It is impossible to instantiate a RawWebImage when not running on the web',
    );
  }
}
