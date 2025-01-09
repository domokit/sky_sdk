// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'icon.dart';
/// @docImport 'image_icon.dart';
library;

import 'basic.dart';
import 'framework.dart';
import 'icon_theme.dart';
import 'icon_theme_data.dart';
import 'implicit_animations.dart';

/// The Flutter logo, in widget form. This widget respects the [IconTheme].
/// For guidelines on using the Flutter logo, visit https://flutter.dev/brand.
///
/// {@youtube 560 315 https://www.youtube.com/watch?v=aAmP-WcI6dg}
///
/// See also:
///
///  * [IconTheme], which provides ambient configuration for icons.
///  * [Icon], for showing icons the Material design icon library.
///  * [ImageIcon], for showing icons from [AssetImage]s or other [ImageProvider]s.
class FlutterLogo extends StatelessWidget {
  /// Creates a widget that paints the Flutter logo.
  ///
  /// The [size] defaults to the value given by the current [IconTheme].
  ///
  /// The [textColor], [style], [duration], and [curve] arguments must not be
  /// null.
  const FlutterLogo({
    super.key,
    this.size,
    this.textColor = const Color(0xFF757575),
    this.style = FlutterLogoStyle.markOnly,
    this.duration = const Duration(milliseconds: 750),
    this.curve = Curves.fastOutSlowIn,
  });

  /// The size of the logo in logical pixels.
  ///
  /// The logo will be fit into a square this size.
  ///
  /// Defaults to the current [IconTheme] size, if any. If there is no
  /// [IconTheme], or it does not specify an explicit size, then it defaults to
  /// 24.0.
  final double? size;

  /// The color used to paint the "Flutter" text on the logo, if [style] is
  /// [FlutterLogoStyle.horizontal] or [FlutterLogoStyle.stacked].
  ///
  /// If possible, the default (a medium grey) should be used against a white
  /// background.
  final Color textColor;

  /// Whether and where to draw the "Flutter" text. By default, only the logo
  /// itself is drawn.
  final FlutterLogoStyle style;

  /// The length of time for the animation if the [style] or [textColor]
  /// properties are changed.
  final Duration duration;

  /// The curve for the logo animation if the [style] or [textColor] change.
  final Curve curve;

  @override
  Widget build(BuildContext context) {
    final IconThemeData iconTheme = IconTheme.of(context);
    final double? iconSize = size ?? iconTheme.size;
    return AnimatedContainer(
      width: iconSize,
      height: iconSize,
      duration: duration,
      curve: curve,
      decoration: FlutterLogoDecoration(style: style, textColor: textColor),
    );
  }
}
