// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/material.dart';
///
/// @docImport 'container.dart';
/// @docImport 'scroll_view.dart';
library;

import 'package:flutter/rendering.dart';

import 'basic.dart';
import 'framework.dart';
import 'image.dart';

/// A sliver widget that paints a [Decoration] either before or after its child
/// paints.
///
/// Unlike [DecoratedBox], this widget expects its child to be a sliver, and
/// must be placed in a widget that expects a sliver.
///
/// If the child sliver has infinite [SliverGeometry.scrollExtent], then we only
/// draw the decoration down to the bottom [SliverGeometry.cacheExtent], and
/// it is necessary to ensure that the bottom border does not creep
/// above the top of the bottom cache. This can happen if the bottom has a
/// border radius larger than the extent of the cache area.
///
/// Commonly used with [BoxDecoration].
///
///
/// {@tool dartpad}
/// This sample shows a radial gradient that draws a moon on a night sky:
///
/// ** See code in examples/api/lib/widgets/sliver/decorated_sliver.0.dart **
/// {@end-tool}
///
/// {@tool dartpad}
/// This example demonstrates how the [CustomScrollView.clipBehavior]
/// impacts a decorated sliver's appearance.
///
/// The [Switch] determines whether clipping is enabled, and
/// the [Slider] adjusts the height of window.
///
/// ** See code in examples/api/lib/widgets/sliver/decorated_sliver.1.dart **
/// {@end-tool}
///
/// This widget does not apply any additional clipping to its [child].
/// To clip a child based on the [Decoration]'s shape, consider using
/// a [ClipPath] widget.
///
/// See also:
///
///  * [DecoratedBox], the version of this class that works with RenderBox widgets.
///  * [Decoration], which you can extend to provide other effects with
///    [DecoratedSliver].
///  * [CustomPaint], another way to draw custom effects from the widget layer.
class DecoratedSliver extends SingleChildRenderObjectWidget {
  /// Creates a widget that paints a [Decoration].
  ///
  /// By default the decoration paints behind the child.
  const DecoratedSliver({
    super.key,
    required this.decoration,
    this.position = DecorationPosition.background,
    Widget? sliver,
  }) : super(child: sliver);

  /// What decoration to paint.
  ///
  /// Commonly a [BoxDecoration].
  final Decoration decoration;

  /// Whether to paint the box decoration behind or in front of the child.
  final DecorationPosition position;

  @override
  RenderDecoratedSliver createRenderObject(BuildContext context) {
    return RenderDecoratedSliver(
      decoration: decoration,
      position: position,
      configuration: createLocalImageConfiguration(context),
    );
  }

  @override
  void updateRenderObject(BuildContext context, RenderDecoratedSliver renderObject) {
    renderObject
      ..decoration = decoration
      ..position = position
      ..configuration = createLocalImageConfiguration(context);
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    final String label = switch (position) {
      DecorationPosition.background => 'bg',
      DecorationPosition.foreground => 'fg',
    };
    properties.add(
      EnumProperty<DecorationPosition>('position', position, level: DiagnosticLevel.hidden),
    );
    properties.add(DiagnosticsProperty<Decoration>(label, decoration));
  }
}
