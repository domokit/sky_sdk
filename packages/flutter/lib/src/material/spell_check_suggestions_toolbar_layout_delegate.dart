// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'spell_check_suggestions_toolbar.dart';
library;

import 'package:flutter/rendering.dart';
import 'package:flutter/widgets.dart' show TextSelectionToolbarLayoutDelegate;

/// Positions the toolbar below [anchor] or adjusts it higher to fit above
/// the bottom view insets, if applicable.
///
/// See also:
///
///   * [SpellCheckSuggestionsToolbar], which uses this to position itself.
class SpellCheckSuggestionsToolbarLayoutDelegate extends SingleChildLayoutDelegate {
  /// Creates an instance of [SpellCheckSuggestionsToolbarLayoutDelegate].
  SpellCheckSuggestionsToolbarLayoutDelegate({required this.anchor});

  /// {@macro flutter.material.SpellCheckSuggestionsToolbar.anchor}
  ///
  /// Should be provided in local coordinates.
  final Offset anchor;

  @override
  BoxConstraints getConstraintsForChild(BoxConstraints constraints) {
    return constraints.loosen();
  }

  @override
  Offset getPositionForChild(Size size, Size childSize) {
    return Offset(
      TextSelectionToolbarLayoutDelegate.centerOn(anchor.dx, childSize.width, size.width),
      // Positions child (of childSize) just enough upwards to fit within size
      // if it otherwise does not fit below the anchor.
      anchor.dy + childSize.height > size.height ? size.height - childSize.height : anchor.dy,
    );
  }

  @override
  bool shouldRelayout(SpellCheckSuggestionsToolbarLayoutDelegate oldDelegate) {
    return anchor != oldDelegate.anchor;
  }
}
