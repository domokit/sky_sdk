// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/material.dart';
///
/// @docImport 'adaptive_text_selection_toolbar.dart';
/// @docImport 'desktop_text_selection_toolbar_button.dart';
library;

import 'dart:ui';

import 'package:flutter/widgets.dart';

import 'colors.dart';

// The minimum padding from all edges of the selection toolbar to all edges of
// the screen.
const double _kToolbarScreenPadding = 8.0;

// These values were measured from a screenshot of the native context menu on
// macOS 13.2 on a Macbook Pro.
const double _kToolbarSaturationBoost = 3;
const double _kToolbarBlurSigma = 20;
const double _kToolbarWidth = 222.0;
const Radius _kToolbarBorderRadius = Radius.circular(8.0);
const EdgeInsets _kToolbarPadding = EdgeInsets.all(6.0);
const List<BoxShadow> _kToolbarShadow = <BoxShadow>[
  BoxShadow(
    color: Color.fromARGB(60, 0, 0, 0),
    blurRadius: 10.0,
    spreadRadius: 0.5,
    offset: Offset(0.0, 4.0),
  ),
];

// These values were measured from a screenshot of the native context menu on
// macOS 13.2 on a Macbook Pro.
const CupertinoDynamicColor _kToolbarBorderColor = CupertinoDynamicColor.withBrightness(
  color: Color(0xFFB8B8B8),
  darkColor: Color(0xFF5B5B5B),
);
const CupertinoDynamicColor _kToolbarBackgroundColor = CupertinoDynamicColor.withBrightness(
  color: Color(0xB2FFFFFF),
  darkColor: Color(0xB2303030),
);

/// A macOS-style text selection toolbar.
///
/// Typically displays buttons for text manipulation, e.g. copying and pasting
/// text.
///
/// Tries to position itself as closely as possible to [anchor] while remaining
/// fully inside the viewport.
///
/// See also:
///
///  * [CupertinoAdaptiveTextSelectionToolbar], where this is used to build the
///    toolbar for desktop platforms.
///  * [AdaptiveTextSelectionToolbar], where this is used to build the toolbar on
///    macOS.
///  * [DesktopTextSelectionToolbar], which is similar but builds a
///    Material-style desktop toolbar.
class CupertinoDesktopTextSelectionToolbar extends StatelessWidget {
  /// Creates a const instance of CupertinoTextSelectionToolbar.
  const CupertinoDesktopTextSelectionToolbar({
    super.key,
    required this.anchor,
    required this.children,
  }) : assert(children.length > 0);

  /// Creates a 5x5 matrix that increases saturation when used with [ColorFilter.matrix].
  ///
  /// The numbers were taken from this comment:
  /// [Cupertino blurs should boost saturation](https://github.com/flutter/flutter/issues/29483#issuecomment-477334981).
  static List<double> _matrixWithSaturation(double saturation) {
    final double r = 0.213 * (1 - saturation);
    final double g = 0.715 * (1 - saturation);
    final double b = 0.072 * (1 - saturation);

    return <double>[
      r + saturation, g, b, 0, 0, //
      r, g + saturation, b, 0, 0, //
      r, g, b + saturation, 0, 0, //
      0, 0, 0, 1, 0, //
    ];
  }

  /// {@macro flutter.material.DesktopTextSelectionToolbar.anchor}
  final Offset anchor;

  /// {@macro flutter.material.TextSelectionToolbar.children}
  ///
  /// See also:
  ///   * [CupertinoDesktopTextSelectionToolbarButton], which builds a default
  ///     macOS-style text selection toolbar text button.
  final List<Widget> children;

  // Builds a toolbar just like the default Mac toolbar, with the right color
  // background, padding, and rounded corners.
  static Widget _defaultToolbarBuilder(BuildContext context, Widget child) {
    return Container(
      width: _kToolbarWidth,
      clipBehavior: Clip.hardEdge,
      decoration: const BoxDecoration(
        boxShadow: _kToolbarShadow,
        borderRadius: BorderRadius.all(_kToolbarBorderRadius),
      ),
      child: BackdropFilter(
        filter: ImageFilter.compose(
          outer: ColorFilter.matrix(_matrixWithSaturation(_kToolbarSaturationBoost)),
          inner: ImageFilter.blur(sigmaX: _kToolbarBlurSigma, sigmaY: _kToolbarBlurSigma),
        ),
        child: DecoratedBox(
          decoration: BoxDecoration(
            color: _kToolbarBackgroundColor.resolveFrom(context),
            border: Border.all(color: _kToolbarBorderColor.resolveFrom(context)),
            borderRadius: const BorderRadius.all(_kToolbarBorderRadius),
          ),
          child: Padding(padding: _kToolbarPadding, child: child),
        ),
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    assert(debugCheckHasMediaQuery(context));

    final double paddingAbove = MediaQuery.paddingOf(context).top + _kToolbarScreenPadding;
    final Offset localAdjustment = Offset(_kToolbarScreenPadding, paddingAbove);

    return Padding(
      padding: EdgeInsets.fromLTRB(
        _kToolbarScreenPadding,
        paddingAbove,
        _kToolbarScreenPadding,
        _kToolbarScreenPadding,
      ),
      child: CustomSingleChildLayout(
        delegate: DesktopTextSelectionToolbarLayoutDelegate(anchor: anchor - localAdjustment),
        child: _defaultToolbarBuilder(
          context,
          Column(mainAxisSize: MainAxisSize.min, children: children),
        ),
      ),
    );
  }
}
