// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'drawer.dart';
/// @docImport 'material.dart';
/// @docImport 'user_accounts_drawer_header.dart';
library;

import 'package:flutter/widgets.dart';

import 'debug.dart';
import 'divider.dart';
import 'theme.dart';

const double _kDrawerHeaderHeight = 160.0 + 1.0; // bottom edge

/// The top-most region of a Material Design drawer. The header's [child]
/// widget, if any, is placed inside a [Container] whose [decoration] can be
/// passed as an argument, inset by the given [padding].
///
/// Part of the Material Design [Drawer].
///
/// Requires one of its ancestors to be a [Material] widget. This condition is
/// satisfied by putting the [DrawerHeader] in a [Drawer].
///
/// See also:
///
///  * [UserAccountsDrawerHeader], a variant of [DrawerHeader] that is
///    specialized for showing user accounts.
///  * <https://material.io/design/components/navigation-drawer.html>
class DrawerHeader extends StatelessWidget {
  /// Creates a Material Design drawer header.
  ///
  /// Requires one of its ancestors to be a [Material] widget.
  const DrawerHeader({
    super.key,
    this.decoration,
    this.margin = const EdgeInsets.only(bottom: 8.0),
    this.padding = const EdgeInsets.fromLTRB(16.0, 16.0, 16.0, 8.0),
    this.duration = const Duration(milliseconds: 250),
    this.curve = Curves.fastOutSlowIn,
    required this.child,
  });

  /// Decoration for the main drawer header [Container]; useful for applying
  /// backgrounds.
  ///
  /// This decoration will extend under the system status bar.
  ///
  /// If this is changed, it will be animated according to [duration] and [curve].
  final Decoration? decoration;

  /// The padding by which to inset [child].
  ///
  /// The [DrawerHeader] additionally offsets the child by the height of the
  /// system status bar.
  ///
  /// If the child is null, the padding has no effect.
  final EdgeInsetsGeometry padding;

  /// The margin around the drawer header.
  final EdgeInsetsGeometry? margin;

  /// The duration for animations of the [decoration].
  final Duration duration;

  /// The curve for animations of the [decoration].
  final Curve curve;

  /// A widget to be placed inside the drawer header, inset by the [padding].
  ///
  /// This widget will be sized to the size of the header. To position the child
  /// precisely, consider using an [Align] or [Center] widget.
  ///
  /// {@macro flutter.widgets.ProxyWidget.child}
  final Widget? child;

  @override
  Widget build(BuildContext context) {
    assert(debugCheckHasMaterial(context));
    assert(debugCheckHasMediaQuery(context));
    final ThemeData theme = Theme.of(context);
    final double statusBarHeight = MediaQuery.paddingOf(context).top;
    return Container(
      height: statusBarHeight + _kDrawerHeaderHeight,
      margin: margin,
      decoration: BoxDecoration(border: Border(bottom: Divider.createBorderSide(context))),
      child: AnimatedContainer(
        padding: padding.add(EdgeInsets.only(top: statusBarHeight)),
        decoration: decoration,
        duration: duration,
        curve: curve,
        child:
            child == null
                ? null
                : DefaultTextStyle(
                  style: theme.textTheme.bodyLarge!,
                  child: MediaQuery.removePadding(context: context, removeTop: true, child: child!),
                ),
      ),
    );
  }
}
