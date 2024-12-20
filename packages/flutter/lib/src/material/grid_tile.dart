// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'grid_tile_bar.dart';
library;

import 'package:flutter/widgets.dart';

/// A tile in a Material Design grid list.
///
/// A grid list is a [GridView] of tiles in a vertical and horizontal
/// array. Each tile typically contains some visually rich content (e.g., an
/// image) together with a [GridTileBar] in either a [header] or a [footer].
///
/// See also:
///
///  * [GridView], which is a scrollable grid of tiles.
///  * [GridTileBar], which is typically used in either the [header] or
///    [footer].
///  * <https://material.io/design/components/image-lists.html>
class GridTile extends StatelessWidget {
  /// Creates a grid tile.
  ///
  /// Must have a child. Does not typically have both a header and a footer.
  const GridTile({super.key, this.header, this.footer, required this.child});

  /// The widget to show over the top of this grid tile.
  ///
  /// Typically a [GridTileBar].
  final Widget? header;

  /// The widget to show over the bottom of this grid tile.
  ///
  /// Typically a [GridTileBar].
  final Widget? footer;

  /// The widget that fills the tile.
  ///
  /// {@macro flutter.widgets.ProxyWidget.child}
  final Widget child;

  @override
  Widget build(BuildContext context) {
    if (header == null && footer == null) {
      return child;
    }

    return Stack(
      children: <Widget>[
        Positioned.fill(child: child),
        if (header != null) Positioned(top: 0.0, left: 0.0, right: 0.0, child: header!),
        if (footer != null) Positioned(left: 0.0, bottom: 0.0, right: 0.0, child: footer!),
      ],
    );
  }
}
