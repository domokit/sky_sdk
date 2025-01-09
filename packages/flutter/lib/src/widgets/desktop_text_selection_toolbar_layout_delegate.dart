// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/cupertino.dart';
/// @docImport 'package:flutter/material.dart';
///
/// @docImport 'text_selection_toolbar_layout_delegate.dart';
library;

import 'package:flutter/rendering.dart';

/// Positions the toolbar at [anchor] if it fits, otherwise moves it so that it
/// just fits fully on-screen.
///
/// See also:
///
///   * [desktopTextSelectionControls], which uses this to position
///     itself.
///   * [cupertinoDesktopTextSelectionControls], which uses this to position
///     itself.
///   * [TextSelectionToolbarLayoutDelegate], which does a similar layout for
///     the mobile text selection toolbars.
class DesktopTextSelectionToolbarLayoutDelegate extends SingleChildLayoutDelegate {
  /// Creates an instance of TextSelectionToolbarLayoutDelegate.
  DesktopTextSelectionToolbarLayoutDelegate({required this.anchor});

  /// The point at which to render the menu, if possible.
  ///
  /// Should be provided in local coordinates.
  final Offset anchor;

  @override
  BoxConstraints getConstraintsForChild(BoxConstraints constraints) {
    return constraints.loosen();
  }

  @override
  Offset getPositionForChild(Size size, Size childSize) {
    final Offset overhang = Offset(
      anchor.dx + childSize.width - size.width,
      anchor.dy + childSize.height - size.height,
    );
    return Offset(
      overhang.dx > 0.0 ? anchor.dx - overhang.dx : anchor.dx,
      overhang.dy > 0.0 ? anchor.dy - overhang.dy : anchor.dy,
    );
  }

  @override
  bool shouldRelayout(DesktopTextSelectionToolbarLayoutDelegate oldDelegate) {
    return anchor != oldDelegate.anchor;
  }
}
