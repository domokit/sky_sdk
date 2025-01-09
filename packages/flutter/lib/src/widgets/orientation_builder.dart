// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'basic.dart';
import 'framework.dart';
import 'layout_builder.dart';
import 'media_query.dart';

/// Signature for a function that builds a widget given an [Orientation].
///
/// Used by [OrientationBuilder.builder].
typedef OrientationWidgetBuilder = Widget Function(BuildContext context, Orientation orientation);

/// Builds a widget tree that can depend on the parent widget's orientation
/// (distinct from the device orientation).
///
/// See also:
///
///  * [LayoutBuilder], which exposes the complete constraints, not just the
///    orientation.
///  * [CustomSingleChildLayout], which positions its child during layout.
///  * [CustomMultiChildLayout], with which you can define the precise layout
///    of a list of children during the layout phase.
///  * [MediaQueryData.orientation], which exposes whether the device is in
///    landscape or portrait mode.
class OrientationBuilder extends StatelessWidget {
  /// Creates an orientation builder.
  const OrientationBuilder({super.key, required this.builder});

  /// Builds the widgets below this widget given this widget's orientation.
  ///
  /// A widget's orientation is a factor of its width relative to its
  /// height. For example, a [Column] widget will have a landscape orientation
  /// if its width exceeds its height, even though it displays its children in
  /// a vertical array.
  final OrientationWidgetBuilder builder;

  Widget _buildWithConstraints(BuildContext context, BoxConstraints constraints) {
    // If the constraints are fully unbounded (i.e., maxWidth and maxHeight are
    // both infinite), we prefer Orientation.portrait because its more common to
    // scroll vertically then horizontally.
    final Orientation orientation =
        constraints.maxWidth > constraints.maxHeight ? Orientation.landscape : Orientation.portrait;
    return builder(context, orientation);
  }

  @override
  Widget build(BuildContext context) {
    return LayoutBuilder(builder: _buildWithConstraints);
  }
}
