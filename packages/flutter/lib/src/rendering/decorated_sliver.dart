// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'object.dart';
import 'proxy_box.dart';
import 'proxy_sliver.dart';
import 'sliver.dart';

/// Paints a [Decoration] either before or after its child paints.
///
/// If the child has infinite scroll extent, then the [Decoration] paints itself up to the
/// bottom cache extent.
class RenderDecoratedSliver extends RenderProxySliver {
  /// Creates a decorated sliver.
  ///
  /// The [decoration], [position], and [configuration] arguments must not be
  /// null. By default the decoration paints behind the child.
  ///
  /// The [ImageConfiguration] will be passed to the decoration (with the size
  /// filled in) to let it resolve images.
  RenderDecoratedSliver({
    required Decoration decoration,
    DecorationPosition position = DecorationPosition.background,
    ImageConfiguration configuration = ImageConfiguration.empty,
  }) : _decoration = decoration,
       _position = position,
       _configuration = configuration;

  /// What decoration to paint.
  ///
  /// Commonly a [BoxDecoration].
  Decoration get decoration => _decoration;
  Decoration _decoration;
  set decoration(Decoration value) {
    if (value == decoration) {
      return;
    }
    _decoration = value;
    _painter?.dispose();
    _painter = decoration.createBoxPainter(markNeedsPaint);
    markNeedsPaint();
  }

  /// Whether to paint the box decoration behind or in front of the child.
  DecorationPosition get position => _position;
  DecorationPosition _position;
  set position(DecorationPosition value) {
    if (value == position) {
      return;
    }
    _position = value;
    markNeedsPaint();
  }

  /// The settings to pass to the decoration when painting, so that it can
  /// resolve images appropriately. See [ImageProvider.resolve] and
  /// [BoxPainter.paint].
  ///
  /// The [ImageConfiguration.textDirection] field is also used by
  /// direction-sensitive [Decoration]s for painting and hit-testing.
  ImageConfiguration get configuration => _configuration;
  ImageConfiguration _configuration;
  set configuration(ImageConfiguration value) {
    if (value == configuration) {
      return;
    }
    _configuration = value;
    markNeedsPaint();
  }

  BoxPainter? _painter;

  @override
  void attach(covariant PipelineOwner owner) {
    _painter = decoration.createBoxPainter(markNeedsPaint);
    super.attach(owner);
  }

  @override
  void detach() {
    _painter?.dispose();
    _painter = null;
    super.detach();
  }

  @override
  void dispose() {
    _painter?.dispose();
    _painter = null;
    super.dispose();
  }

  @override
  void paint(PaintingContext context, Offset offset) {
    if (child == null || !child!.geometry!.visible) {
      return;
    }
    // In the case where the child sliver has infinite scroll extent, the decoration
    // should only extend down to the bottom cache extent.
    final double cappedMainAxisExtent =
        child!.geometry!.scrollExtent.isInfinite
            ? constraints.scrollOffset + child!.geometry!.cacheExtent + constraints.cacheOrigin
            : child!.geometry!.scrollExtent;
    final (Size childSize, Offset scrollOffset) = switch (constraints.axis) {
      Axis.horizontal => (
        Size(cappedMainAxisExtent, constraints.crossAxisExtent),
        Offset(-constraints.scrollOffset, 0.0),
      ),
      Axis.vertical => (
        Size(constraints.crossAxisExtent, cappedMainAxisExtent),
        Offset(0.0, -constraints.scrollOffset),
      ),
    };
    offset += (child!.parentData! as SliverPhysicalParentData).paintOffset;
    void paintDecoration() => _painter!.paint(
      context.canvas,
      offset + scrollOffset,
      configuration.copyWith(size: childSize),
    );
    switch (position) {
      case DecorationPosition.background:
        paintDecoration();
        context.paintChild(child!, offset);
      case DecorationPosition.foreground:
        context.paintChild(child!, offset);
        paintDecoration();
    }
  }
}
