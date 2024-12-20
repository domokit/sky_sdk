// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'adaptive_text_selection_toolbar.dart';
/// @docImport 'desktop_text_selection_toolbar_button.dart';
library;

import 'package:flutter/widgets.dart';

import 'material.dart';
import 'text_selection_toolbar.dart';

// These values were measured from a screenshot of TextEdit on macOS 10.15.7 on
// a Macbook Pro.
const double _kToolbarScreenPadding = 8.0;
const double _kToolbarWidth = 222.0;

/// A Material-style desktop text selection toolbar.
///
/// Typically displays buttons for text manipulation, e.g. copying and pasting
/// text.
///
/// Tries to position its top left corner as closely as possible to [anchor]
/// while remaining fully inside the viewport.
///
/// See also:
///
///  * [AdaptiveTextSelectionToolbar], which builds the toolbar for the current
///    platform.
///  * [TextSelectionToolbar], which is similar, but builds an Android-style
///    toolbar.
class DesktopTextSelectionToolbar extends StatelessWidget {
  /// Creates a const instance of DesktopTextSelectionToolbar.
  const DesktopTextSelectionToolbar({super.key, required this.anchor, required this.children})
    : assert(children.length > 0);

  /// {@template flutter.material.DesktopTextSelectionToolbar.anchor}
  /// The point where the toolbar will attempt to position itself as closely as
  /// possible.
  /// {@endtemplate}
  final Offset anchor;

  /// {@macro flutter.material.TextSelectionToolbar.children}
  ///
  /// See also:
  ///   * [DesktopTextSelectionToolbarButton], which builds a default
  ///     Material-style desktop text selection toolbar text button.
  final List<Widget> children;

  // Builds a desktop toolbar in the Material style.
  static Widget _defaultToolbarBuilder(BuildContext context, Widget child) {
    return SizedBox(
      width: _kToolbarWidth,
      child: Material(
        borderRadius: const BorderRadius.all(Radius.circular(7.0)),
        clipBehavior: Clip.antiAlias,
        elevation: 1.0,
        type: MaterialType.card,
        child: child,
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
