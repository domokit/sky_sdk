// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/material.dart';
/// @docImport 'package:flutter/rendering.dart';
///
/// @docImport 'text_span.dart';
/// @docImport 'text_style.dart';
library;

import 'dart:ui' show TextDirection;

export 'dart:ui'
    show
        BlendMode,
        BlurStyle,
        Canvas,
        Clip,
        Color,
        ColorFilter,
        FilterQuality,
        FontFeature,
        FontStyle,
        FontVariation,
        FontWeight,
        GlyphInfo,
        ImageShader,
        Locale,
        MaskFilter,
        Offset,
        Paint,
        PaintingStyle,
        Path,
        PathFillType,
        PathOperation,
        RRect,
        RSTransform,
        Radius,
        Rect,
        Shader,
        Shadow,
        Size,
        StrokeCap,
        StrokeJoin,
        TextAffinity,
        TextAlign,
        TextBaseline,
        TextBox,
        TextDecoration,
        TextDecorationStyle,
        TextDirection,
        TextHeightBehavior,
        TextLeadingDistribution,
        TextPosition,
        TileMode,
        VertexMode;

export 'package:flutter/foundation.dart' show VoidCallback;

// Intentionally not exported:
//  - Image, instantiateImageCodec, decodeImageFromList:
//      We use ui.* to make it very explicit that these are low-level image APIs.
//      Generally, higher layers provide more reasonable APIs around images.
//  - lerpDouble:
//      Hopefully this will eventually become Double.lerp.
//  - Paragraph, ParagraphBuilder, ParagraphStyle, TextBox:
//      These are low-level text primitives. Use this package's TextPainter API.
//  - Picture, PictureRecorder, Scene, SceneBuilder:
//      These are low-level primitives. Generally, the rendering layer makes these moot.
//  - Gradient:
//      Use this package's higher-level Gradient API instead.
//  - window, WindowPadding
//      These are generally wrapped by other APIs so we always refer to them directly
//      as ui.* to avoid making them seem like high-level APIs.

/// The description of the difference between two objects, in the context of how
/// it will affect the rendering.
///
/// Used by [TextSpan.compareTo] and [TextStyle.compareTo].
///
/// The values in this enum are ordered such that they are in increasing order
/// of cost. A value with index N implies all the values with index less than N.
/// For example, [layout] (index 3) implies [paint] (2).
enum RenderComparison {
  /// The two objects are identical (meaning deeply equal, not necessarily
  /// [dart:core.identical]).
  identical,

  /// The two objects are identical for the purpose of layout, but may be different
  /// in other ways.
  ///
  /// For example, maybe some event handlers changed.
  metadata,

  /// The two objects are different but only in ways that affect paint, not layout.
  ///
  /// For example, only the color is changed.
  ///
  /// [RenderObject.markNeedsPaint] would be necessary to handle this kind of
  /// change in a render object.
  paint,

  /// The two objects are different in ways that affect layout (and therefore paint).
  ///
  /// For example, the size is changed.
  ///
  /// This is the most drastic level of change possible.
  ///
  /// [RenderObject.markNeedsLayout] would be necessary to handle this kind of
  /// change in a render object.
  layout,
}

/// The two cardinal directions in two dimensions.
///
/// The axis is always relative to the current coordinate space. This means, for
/// example, that a [horizontal] axis might actually be diagonally from top
/// right to bottom left, due to some local [Transform] applied to the scene.
///
/// See also:
///
///  * [AxisDirection], which is a directional version of this enum (with values
///    like left and right, rather than just horizontal).
///  * [TextDirection], which disambiguates between left-to-right horizontal
///    content and right-to-left horizontal content.
enum Axis {
  /// Left and right.
  ///
  /// See also:
  ///
  ///  * [TextDirection], which disambiguates between left-to-right horizontal
  ///    content and right-to-left horizontal content.
  horizontal,

  /// Up and down.
  vertical,
}

/// Returns the opposite of the given [Axis].
///
/// Specifically, returns [Axis.horizontal] for [Axis.vertical], and
/// vice versa.
///
/// See also:
///
///  * [flipAxisDirection], which does the same thing for [AxisDirection] values.
Axis flipAxis(Axis direction) {
  return switch (direction) {
    Axis.horizontal => Axis.vertical,
    Axis.vertical => Axis.horizontal,
  };
}

/// A direction in which boxes flow vertically.
///
/// This is used by the flex algorithm (e.g. [Column]) to decide in which
/// direction to draw boxes.
///
/// This is also used to disambiguate `start` and `end` values (e.g.
/// [MainAxisAlignment.start] or [CrossAxisAlignment.end]).
///
/// See also:
///
///  * [TextDirection], which controls the same thing but horizontally.
enum VerticalDirection {
  /// Boxes should start at the bottom and be stacked vertically towards the top.
  ///
  /// The "start" is at the bottom, the "end" is at the top.
  up,

  /// Boxes should start at the top and be stacked vertically towards the bottom.
  ///
  /// The "start" is at the top, the "end" is at the bottom.
  down,
}

/// A direction along either the horizontal or vertical [Axis] in which the
/// origin, or zero position, is determined.
///
/// This value relates to the direction in which the scroll offset increases
/// from the origin. This value does not represent the direction of user input
/// that may be modifying the scroll offset, such as from a drag. For the
/// active scrolling direction, see [ScrollDirection].
///
/// {@tool dartpad}
/// This sample shows a [CustomScrollView], with [Radio] buttons in the
/// [AppBar.bottom] that change the [AxisDirection] to illustrate different
/// configurations.
///
/// ** See code in examples/api/lib/painting/axis_direction/axis_direction.0.dart **
/// {@end-tool}
///
/// See also:
///
///   * [ScrollDirection], the direction of active scrolling, relative to the positive
///     scroll offset axis given by an [AxisDirection] and a [GrowthDirection].
///   * [GrowthDirection], the direction in which slivers and their content are
///     ordered, relative to the scroll offset axis as specified by
///     [AxisDirection].
///   * [CustomScrollView.anchor], the relative position of the zero scroll
///     offset in a viewport and inflection point for [AxisDirection]s of the
///     same cardinal [Axis].
///   * [axisDirectionIsReversed], which returns whether traveling along the
///     given axis direction visits coordinates along that axis in numerically
///     decreasing order.
enum AxisDirection {
  /// A direction in the [Axis.vertical] where zero is at the bottom and
  /// positive values are above it: `⇈`
  ///
  /// Alphabetical content with a [GrowthDirection.forward] would have the A at
  /// the bottom and the Z at the top.
  ///
  /// For example, the behavior of a [ListView] with [ListView.reverse] set to
  /// true would have this axis direction.
  ///
  /// See also:
  ///
  ///   * [axisDirectionIsReversed], which returns whether traveling along the
  ///     given axis direction visits coordinates along that axis in numerically
  ///     decreasing order.
  up,

  /// A direction in the [Axis.horizontal] where zero is on the left and
  /// positive values are to the right of it: `⇉`
  ///
  /// Alphabetical content with a [GrowthDirection.forward] would have the A on
  /// the left and the Z on the right. This is the ordinary reading order for a
  /// horizontal set of tabs in an English application, for example.
  ///
  /// For example, the behavior of a [ListView] with [ListView.scrollDirection]
  /// set to [Axis.horizontal] would have this axis direction.
  ///
  /// See also:
  ///
  ///   * [axisDirectionIsReversed], which returns whether traveling along the
  ///     given axis direction visits coordinates along that axis in numerically
  ///     decreasing order.
  right,

  /// A direction in the [Axis.vertical] where zero is at the top and positive
  /// values are below it: `⇊`
  ///
  /// Alphabetical content with a [GrowthDirection.forward] would have the A at
  /// the top and the Z at the bottom. This is the ordinary reading order for a
  /// vertical list.
  ///
  /// For example, the default behavior of a [ListView] would have this axis
  /// direction.
  ///
  /// See also:
  ///
  ///   * [axisDirectionIsReversed], which returns whether traveling along the
  ///     given axis direction visits coordinates along that axis in numerically
  ///     decreasing order.
  down,

  /// A direction in the [Axis.horizontal] where zero is to the right and
  /// positive values are to the left of it: `⇇`
  ///
  /// Alphabetical content with a [GrowthDirection.forward] would have the A at
  /// the right and the Z at the left. This is the ordinary reading order for a
  /// horizontal set of tabs in a Hebrew application, for example.
  ///
  /// For example, the behavior of a [ListView] with [ListView.scrollDirection]
  /// set to [Axis.horizontal] and [ListView.reverse] set to true would have
  /// this axis direction.
  ///
  /// See also:
  ///
  ///   * [axisDirectionIsReversed], which returns whether traveling along the
  ///     given axis direction visits coordinates along that axis in numerically
  ///     decreasing order.
  left,
}

/// Returns the [Axis] that contains the given [AxisDirection].
///
/// Specifically, returns [Axis.vertical] for [AxisDirection.up] and
/// [AxisDirection.down] and returns [Axis.horizontal] for [AxisDirection.left]
/// and [AxisDirection.right].
Axis axisDirectionToAxis(AxisDirection axisDirection) {
  return switch (axisDirection) {
    AxisDirection.up || AxisDirection.down => Axis.vertical,
    AxisDirection.left || AxisDirection.right => Axis.horizontal,
  };
}

/// Returns the [AxisDirection] in which reading occurs in the given [TextDirection].
///
/// Specifically, returns [AxisDirection.left] for [TextDirection.rtl] and
/// [AxisDirection.right] for [TextDirection.ltr].
AxisDirection textDirectionToAxisDirection(TextDirection textDirection) {
  return switch (textDirection) {
    TextDirection.rtl => AxisDirection.left,
    TextDirection.ltr => AxisDirection.right,
  };
}

/// Returns the opposite of the given [AxisDirection].
///
/// Specifically, returns [AxisDirection.up] for [AxisDirection.down] (and
/// vice versa), as well as [AxisDirection.left] for [AxisDirection.right] (and
/// vice versa).
///
/// See also:
///
///  * [flipAxis], which does the same thing for [Axis] values.
AxisDirection flipAxisDirection(AxisDirection axisDirection) {
  return switch (axisDirection) {
    AxisDirection.up => AxisDirection.down,
    AxisDirection.right => AxisDirection.left,
    AxisDirection.down => AxisDirection.up,
    AxisDirection.left => AxisDirection.right,
  };
}

/// Returns whether traveling along the given axis direction visits coordinates
/// along that axis in numerically decreasing order.
///
/// Specifically, returns true for [AxisDirection.up] and [AxisDirection.left]
/// and false for [AxisDirection.down] and [AxisDirection.right].
bool axisDirectionIsReversed(AxisDirection axisDirection) {
  return switch (axisDirection) {
    AxisDirection.up || AxisDirection.left => true,
    AxisDirection.down || AxisDirection.right => false,
  };
}
