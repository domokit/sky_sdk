// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/material.dart';
library;

import 'package:flutter/rendering.dart';

import 'basic.dart';
import 'framework.dart';
import 'icon.dart';
import 'icon_theme.dart';
import 'icon_theme_data.dart';
import 'image.dart';

/// An icon that comes from an [ImageProvider], e.g. an [AssetImage].
///
/// See also:
///
///  * [IconButton], for interactive icons.
///  * [IconTheme], which provides ambient configuration for icons.
///  * [Icon], for icons based on glyphs from fonts instead of images.
///  * [Icons], the library of Material Icons.
class ImageIcon extends StatelessWidget {
  /// Creates an image icon.
  ///
  /// The [size] and [color] default to the value given by the current [IconTheme].
  const ImageIcon(this.image, {super.key, this.size, this.color, this.semanticLabel});

  /// The image to display as the icon.
  ///
  /// The icon can be null, in which case the widget will render as an empty
  /// space of the specified [size].
  final ImageProvider? image;

  /// The size of the icon in logical pixels.
  ///
  /// Icons occupy a square with width and height equal to size.
  ///
  /// Defaults to the current [IconTheme] size, if any. If there is no
  /// [IconTheme], or it does not specify an explicit size, then it defaults to
  /// 24.0.
  final double? size;

  /// The color to use when drawing the icon.
  ///
  /// Defaults to the current [IconTheme] color, if any. If there is
  /// no [IconTheme], then it defaults to not recolorizing the image.
  ///
  /// The image will be additionally adjusted by the opacity of the current
  /// [IconTheme], if any.
  final Color? color;

  /// Semantic label for the icon.
  ///
  /// Announced by assistive technologies (e.g TalkBack/VoiceOver).
  /// This label does not show in the UI.
  ///
  ///  * [SemanticsProperties.label], which is set to [semanticLabel] in the
  ///    underlying	 [Semantics] widget.
  final String? semanticLabel;

  @override
  Widget build(BuildContext context) {
    final IconThemeData iconTheme = IconTheme.of(context);
    final double? iconSize = size ?? iconTheme.size;

    if (image == null) {
      return Semantics(label: semanticLabel, child: SizedBox(width: iconSize, height: iconSize));
    }

    final double? iconOpacity = iconTheme.opacity;
    Color iconColor = color ?? iconTheme.color!;

    if (iconOpacity != null && iconOpacity != 1.0) {
      iconColor = iconColor.withOpacity(iconColor.opacity * iconOpacity);
    }

    return Semantics(
      label: semanticLabel,
      child: Image(
        image: image!,
        width: iconSize,
        height: iconSize,
        color: iconColor,
        fit: BoxFit.scaleDown,
        excludeFromSemantics: true,
      ),
    );
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(
      DiagnosticsProperty<ImageProvider>('image', image, ifNull: '<empty>', showName: false),
    );
    properties.add(DoubleProperty('size', size, defaultValue: null));
    properties.add(ColorProperty('color', color, defaultValue: null));
  }
}
