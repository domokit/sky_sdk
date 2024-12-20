// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/widgets.dart';
///
/// @docImport 'list_wheel_viewport.dart';
/// @docImport 'sliver_fixed_extent_list.dart';
/// @docImport 'sliver_grid.dart';
/// @docImport 'sliver_list.dart';
library;

import 'dart:math' as math;

import 'package:flutter/animation.dart';
import 'package:flutter/foundation.dart';
import 'package:flutter/semantics.dart';

import 'box.dart';
import 'debug.dart';
import 'layer.dart';
import 'object.dart';
import 'sliver.dart';
import 'viewport_offset.dart';

/// The unit of measurement for a [Viewport.cacheExtent].
enum CacheExtentStyle {
  /// Treat the [Viewport.cacheExtent] as logical pixels.
  pixel,

  /// Treat the [Viewport.cacheExtent] as a multiplier of the main axis extent.
  viewport,
}

/// An interface for render objects that are bigger on the inside.
///
/// Some render objects, such as [RenderViewport], present a portion of their
/// content, which can be controlled by a [ViewportOffset]. This interface lets
/// the framework recognize such render objects and interact with them without
/// having specific knowledge of all the various types of viewports.
abstract interface class RenderAbstractViewport extends RenderObject {
  /// Returns the [RenderAbstractViewport] that most tightly encloses the given
  /// render object.
  ///
  /// If the object does not have a [RenderAbstractViewport] as an ancestor,
  /// this function returns null.
  ///
  /// See also:
  ///
  /// * [RenderAbstractViewport.of], which is similar to this method, but
  ///   asserts if no [RenderAbstractViewport] ancestor is found.
  static RenderAbstractViewport? maybeOf(RenderObject? object) {
    while (object != null) {
      if (object is RenderAbstractViewport) {
        return object;
      }
      object = object.parent;
    }
    return null;
  }

  /// Returns the [RenderAbstractViewport] that most tightly encloses the given
  /// render object.
  ///
  /// If the object does not have a [RenderAbstractViewport] as an ancestor,
  /// this function will assert in debug mode, and throw an exception in release
  /// mode.
  ///
  /// See also:
  ///
  /// * [RenderAbstractViewport.maybeOf], which is similar to this method, but
  ///   returns null if no [RenderAbstractViewport] ancestor is found.
  static RenderAbstractViewport of(RenderObject? object) {
    final RenderAbstractViewport? viewport = maybeOf(object);
    assert(() {
      if (viewport == null) {
        throw FlutterError(
          'RenderAbstractViewport.of() was called with a render object that was '
          'not a descendant of a RenderAbstractViewport.\n'
          'No RenderAbstractViewport render object ancestor could be found starting '
          'from the object that was passed to RenderAbstractViewport.of().\n'
          'The render object where the viewport search started was:\n'
          '  $object',
        );
      }
      return true;
    }());
    return viewport!;
  }

  /// Returns the offset that would be needed to reveal the `target`
  /// [RenderObject].
  ///
  /// This is used by [RenderViewportBase.showInViewport], which is
  /// itself used by [RenderObject.showOnScreen] for
  /// [RenderViewportBase], which is in turn used by the semantics
  /// system to implement scrolling for accessibility tools.
  ///
  /// The optional `rect` parameter describes which area of that `target` object
  /// should be revealed in the viewport. If `rect` is null, the entire
  /// `target` [RenderObject] (as defined by its [RenderObject.paintBounds])
  /// will be revealed. If `rect` is provided it has to be given in the
  /// coordinate system of the `target` object.
  ///
  /// The `alignment` argument describes where the target should be positioned
  /// after applying the returned offset. If `alignment` is 0.0, the child must
  /// be positioned as close to the leading edge of the viewport as possible. If
  /// `alignment` is 1.0, the child must be positioned as close to the trailing
  /// edge of the viewport as possible. If `alignment` is 0.5, the child must be
  /// positioned as close to the center of the viewport as possible.
  ///
  /// The `target` might not be a direct child of this viewport but it must be a
  /// descendant of the viewport. Other viewports in between this viewport and
  /// the `target` will not be adjusted.
  ///
  /// This method assumes that the content of the viewport moves linearly, i.e.
  /// when the offset of the viewport is changed by x then `target` also moves
  /// by x within the viewport.
  ///
  /// The optional [Axis] is used by
  /// [RenderTwoDimensionalViewport.getOffsetToReveal] to
  /// determine which of the two axes to compute an offset for. One dimensional
  /// subclasses like [RenderViewportBase] and [RenderListWheelViewport]
  /// will ignore the `axis` value if provided, since there is only one [Axis].
  ///
  /// If the `axis` is omitted when called on [RenderTwoDimensionalViewport],
  /// the [RenderTwoDimensionalViewport.mainAxis] is used. To reveal an object
  /// properly in both axes, this method should be called for each [Axis] as the
  /// returned [RevealedOffset.offset] only represents the offset of one of the
  /// the two [ScrollPosition]s.
  ///
  /// See also:
  ///
  ///  * [RevealedOffset], which describes the return value of this method.
  RevealedOffset getOffsetToReveal(RenderObject target, double alignment, {Rect? rect, Axis? axis});

  /// The default value for the cache extent of the viewport.
  ///
  /// This default assumes [CacheExtentStyle.pixel].
  ///
  /// See also:
  ///
  ///  * [RenderViewportBase.cacheExtent] for a definition of the cache extent.
  static const double defaultCacheExtent = 250.0;
}

/// Return value for [RenderAbstractViewport.getOffsetToReveal].
///
/// It indicates the [offset] required to reveal an element in a viewport and
/// the [rect] position said element would have in the viewport at that
/// [offset].
class RevealedOffset {
  /// Instantiates a return value for [RenderAbstractViewport.getOffsetToReveal].
  const RevealedOffset({required this.offset, required this.rect});

  /// Offset for the viewport to reveal a specific element in the viewport.
  ///
  /// See also:
  ///
  ///  * [RenderAbstractViewport.getOffsetToReveal], which calculates this
  ///    value for a specific element.
  final double offset;

  /// The [Rect] in the outer coordinate system of the viewport at which the
  /// to-be-revealed element would be located if the viewport's offset is set
  /// to [offset].
  ///
  /// A viewport usually has two coordinate systems and works as an adapter
  /// between the two:
  ///
  /// The inner coordinate system has its origin at the top left corner of the
  /// content that moves inside the viewport. The origin of this coordinate
  /// system usually moves around relative to the leading edge of the viewport
  /// when the viewport offset changes.
  ///
  /// The outer coordinate system has its origin at the top left corner of the
  /// visible part of the viewport. This origin stays at the same position
  /// regardless of the current viewport offset.
  ///
  /// In other words: [rect] describes where the revealed element would be
  /// located relative to the top left corner of the visible part of the
  /// viewport if the viewport's offset is set to [offset].
  ///
  /// See also:
  ///
  ///  * [RenderAbstractViewport.getOffsetToReveal], which calculates this
  ///    value for a specific element.
  final Rect rect;

  /// Determines which provided leading or trailing edge of the viewport, as
  /// [RevealedOffset]s, will be used for [RenderViewportBase.showInViewport]
  /// accounting for the size and already visible portion of the [RenderObject]
  /// that is being revealed.
  ///
  /// Also used by [RenderTwoDimensionalViewport.showInViewport] for each
  /// horizontal and vertical [Axis].
  ///
  /// If the target [RenderObject] is already fully visible, this will return
  /// null.
  static RevealedOffset? clampOffset({
    required RevealedOffset leadingEdgeOffset,
    required RevealedOffset trailingEdgeOffset,
    required double currentOffset,
  }) {
    //           scrollOffset
    //                       0 +---------+
    //                         |         |
    //                       _ |         |
    //    viewport position |  |         |
    // with `descendant` at |  |         | _
    //        trailing edge |_ | xxxxxxx |  | viewport position
    //                         |         |  | with `descendant` at
    //                         |         | _| leading edge
    //                         |         |
    //                     800 +---------+
    //
    // `trailingEdgeOffset`: Distance from scrollOffset 0 to the start of the
    //                       viewport on the left in image above.
    // `leadingEdgeOffset`: Distance from scrollOffset 0 to the start of the
    //                      viewport on the right in image above.
    //
    // The viewport position on the left is achieved by setting `offset.pixels`
    // to `trailingEdgeOffset`, the one on the right by setting it to
    // `leadingEdgeOffset`.
    final bool inverted = leadingEdgeOffset.offset < trailingEdgeOffset.offset;
    final RevealedOffset smaller;
    final RevealedOffset larger;
    (smaller, larger) =
        inverted
            ? (leadingEdgeOffset, trailingEdgeOffset)
            : (trailingEdgeOffset, leadingEdgeOffset);
    if (currentOffset > larger.offset) {
      return larger;
    } else if (currentOffset < smaller.offset) {
      return smaller;
    } else {
      return null;
    }
  }

  @override
  String toString() {
    return '${objectRuntimeType(this, 'RevealedOffset')}(offset: $offset, rect: $rect)';
  }
}

/// A base class for render objects that are bigger on the inside.
///
/// This render object provides the shared code for render objects that host
/// [RenderSliver] render objects inside a [RenderBox]. The viewport establishes
/// an [axisDirection], which orients the sliver's coordinate system, which is
/// based on scroll offsets rather than Cartesian coordinates.
///
/// The viewport also listens to an [offset], which determines the
/// [SliverConstraints.scrollOffset] input to the sliver layout protocol.
///
/// Subclasses typically override [performLayout] and call
/// [layoutChildSequence], perhaps multiple times.
///
/// See also:
///
///  * [RenderSliver], which explains more about the Sliver protocol.
///  * [RenderBox], which explains more about the Box protocol.
///  * [RenderSliverToBoxAdapter], which allows a [RenderBox] object to be
///    placed inside a [RenderSliver] (the opposite of this class).
abstract class RenderViewportBase<ParentDataClass extends ContainerParentDataMixin<RenderSliver>>
    extends RenderBox
    with ContainerRenderObjectMixin<RenderSliver, ParentDataClass>
    implements RenderAbstractViewport {
  /// Initializes fields for subclasses.
  ///
  /// The [cacheExtent], if null, defaults to [RenderAbstractViewport.defaultCacheExtent].
  ///
  /// The [cacheExtent] must be specified if [cacheExtentStyle] is not [CacheExtentStyle.pixel].
  RenderViewportBase({
    AxisDirection axisDirection = AxisDirection.down,
    required AxisDirection crossAxisDirection,
    required ViewportOffset offset,
    double? cacheExtent,
    CacheExtentStyle cacheExtentStyle = CacheExtentStyle.pixel,
    Clip clipBehavior = Clip.hardEdge,
  }) : assert(axisDirectionToAxis(axisDirection) != axisDirectionToAxis(crossAxisDirection)),
       assert(cacheExtent != null || cacheExtentStyle == CacheExtentStyle.pixel),
       _axisDirection = axisDirection,
       _crossAxisDirection = crossAxisDirection,
       _offset = offset,
       _cacheExtent = cacheExtent ?? RenderAbstractViewport.defaultCacheExtent,
       _cacheExtentStyle = cacheExtentStyle,
       _clipBehavior = clipBehavior;

  /// Report the semantics of this node, for example for accessibility purposes.
  ///
  /// [RenderViewportBase] adds [RenderViewport.useTwoPaneSemantics] to the
  /// provided [SemanticsConfiguration] to support children using
  /// [RenderViewport.excludeFromScrolling].
  ///
  /// This method should be overridden by subclasses that have interesting
  /// semantic information. Overriding subclasses should call
  /// `super.describeSemanticsConfiguration(config)` to ensure
  /// [RenderViewport.useTwoPaneSemantics] is still added to `config`.
  ///
  /// See also:
  ///
  /// * [RenderObject.describeSemanticsConfiguration], for important
  ///   details about not mutating a [SemanticsConfiguration] out of context.
  @override
  void describeSemanticsConfiguration(SemanticsConfiguration config) {
    super.describeSemanticsConfiguration(config);

    config.addTagForChildren(RenderViewport.useTwoPaneSemantics);
  }

  @override
  void visitChildrenForSemantics(RenderObjectVisitor visitor) {
    childrenInPaintOrder
        .where(
          (RenderSliver sliver) => sliver.geometry!.visible || sliver.geometry!.cacheExtent > 0.0,
        )
        .forEach(visitor);
  }

  /// The direction in which the [SliverConstraints.scrollOffset] increases.
  ///
  /// For example, if the [axisDirection] is [AxisDirection.down], a scroll
  /// offset of zero is at the top of the viewport and increases towards the
  /// bottom of the viewport.
  AxisDirection get axisDirection => _axisDirection;
  AxisDirection _axisDirection;
  set axisDirection(AxisDirection value) {
    if (value == _axisDirection) {
      return;
    }
    _axisDirection = value;
    markNeedsLayout();
  }

  /// The direction in which child should be laid out in the cross axis.
  ///
  /// For example, if the [axisDirection] is [AxisDirection.down], this property
  /// is typically [AxisDirection.left] if the ambient [TextDirection] is
  /// [TextDirection.rtl] and [AxisDirection.right] if the ambient
  /// [TextDirection] is [TextDirection.ltr].
  AxisDirection get crossAxisDirection => _crossAxisDirection;
  AxisDirection _crossAxisDirection;
  set crossAxisDirection(AxisDirection value) {
    if (value == _crossAxisDirection) {
      return;
    }
    _crossAxisDirection = value;
    markNeedsLayout();
  }

  /// The axis along which the viewport scrolls.
  ///
  /// For example, if the [axisDirection] is [AxisDirection.down], then the
  /// [axis] is [Axis.vertical] and the viewport scrolls vertically.
  Axis get axis => axisDirectionToAxis(axisDirection);

  /// Which part of the content inside the viewport should be visible.
  ///
  /// The [ViewportOffset.pixels] value determines the scroll offset that the
  /// viewport uses to select which part of its content to display. As the user
  /// scrolls the viewport, this value changes, which changes the content that
  /// is displayed.
  ViewportOffset get offset => _offset;
  ViewportOffset _offset;
  set offset(ViewportOffset value) {
    if (value == _offset) {
      return;
    }
    if (attached) {
      _offset.removeListener(markNeedsLayout);
    }
    _offset = value;
    if (attached) {
      _offset.addListener(markNeedsLayout);
    }
    // We need to go through layout even if the new offset has the same pixels
    // value as the old offset so that we will apply our viewport and content
    // dimensions.
    markNeedsLayout();
  }

  // TODO(ianh): cacheExtent/cacheExtentStyle should be a single
  // object that specifies both the scalar value and the unit, not a
  // pair of independent setters. Changing that would allow a more
  // rational API and would let us make the getter non-nullable.

  /// {@template flutter.rendering.RenderViewportBase.cacheExtent}
  /// The viewport has an area before and after the visible area to cache items
  /// that are about to become visible when the user scrolls.
  ///
  /// Items that fall in this cache area are laid out even though they are not
  /// (yet) visible on screen. The [cacheExtent] describes how many pixels
  /// the cache area extends before the leading edge and after the trailing edge
  /// of the viewport.
  ///
  /// The total extent, which the viewport will try to cover with children, is
  /// [cacheExtent] before the leading edge + extent of the main axis +
  /// [cacheExtent] after the trailing edge.
  ///
  /// The cache area is also used to implement implicit accessibility scrolling
  /// on iOS: When the accessibility focus moves from an item in the visible
  /// viewport to an invisible item in the cache area, the framework will bring
  /// that item into view with an (implicit) scroll action.
  /// {@endtemplate}
  ///
  /// The getter can never return null, but the field is nullable
  /// because the setter can be set to null to reset the value to
  /// [RenderAbstractViewport.defaultCacheExtent] (in which case
  /// [cacheExtentStyle] must be [CacheExtentStyle.pixel]).
  ///
  /// See also:
  ///
  ///  * [cacheExtentStyle], which controls the units of the [cacheExtent].
  double? get cacheExtent => _cacheExtent;
  double _cacheExtent;
  set cacheExtent(double? value) {
    value ??= RenderAbstractViewport.defaultCacheExtent;
    if (value == _cacheExtent) {
      return;
    }
    _cacheExtent = value;
    markNeedsLayout();
  }

  /// This value is set during layout based on the [CacheExtentStyle].
  ///
  /// When the style is [CacheExtentStyle.viewport], it is the main axis extent
  /// of the viewport multiplied by the requested cache extent, which is still
  /// expressed in pixels.
  double? _calculatedCacheExtent;

  /// {@template flutter.rendering.RenderViewportBase.cacheExtentStyle}
  /// Controls how the [cacheExtent] is interpreted.
  ///
  /// If set to [CacheExtentStyle.pixel], the [cacheExtent] will be
  /// treated as a logical pixels, and the default [cacheExtent] is
  /// [RenderAbstractViewport.defaultCacheExtent].
  ///
  /// If set to [CacheExtentStyle.viewport], the [cacheExtent] will be
  /// treated as a multiplier for the main axis extent of the
  /// viewport. In this case there is no default [cacheExtent]; it
  /// must be explicitly specified.
  /// {@endtemplate}
  ///
  /// Changing the [cacheExtentStyle] without also changing the [cacheExtent]
  /// is rarely the correct choice.
  CacheExtentStyle get cacheExtentStyle => _cacheExtentStyle;
  CacheExtentStyle _cacheExtentStyle;
  set cacheExtentStyle(CacheExtentStyle value) {
    if (value == _cacheExtentStyle) {
      return;
    }
    _cacheExtentStyle = value;
    markNeedsLayout();
  }

  /// {@macro flutter.material.Material.clipBehavior}
  ///
  /// Defaults to [Clip.hardEdge].
  Clip get clipBehavior => _clipBehavior;
  Clip _clipBehavior = Clip.hardEdge;
  set clipBehavior(Clip value) {
    if (value != _clipBehavior) {
      _clipBehavior = value;
      markNeedsPaint();
      markNeedsSemanticsUpdate();
    }
  }

  @override
  void attach(PipelineOwner owner) {
    super.attach(owner);
    _offset.addListener(markNeedsLayout);
  }

  @override
  void detach() {
    _offset.removeListener(markNeedsLayout);
    super.detach();
  }

  /// Throws an exception saying that the object does not support returning
  /// intrinsic dimensions if, in debug mode, we are not in the
  /// [RenderObject.debugCheckingIntrinsics] mode.
  ///
  /// This is used by [computeMinIntrinsicWidth] et al because viewports do not
  /// generally support returning intrinsic dimensions. See the discussion at
  /// [computeMinIntrinsicWidth].
  @protected
  bool debugThrowIfNotCheckingIntrinsics() {
    assert(() {
      if (!RenderObject.debugCheckingIntrinsics) {
        assert(this is! RenderShrinkWrappingViewport); // it has its own message
        throw FlutterError.fromParts(<DiagnosticsNode>[
          ErrorSummary('$runtimeType does not support returning intrinsic dimensions.'),
          ErrorDescription(
            'Calculating the intrinsic dimensions would require instantiating every child of '
            'the viewport, which defeats the point of viewports being lazy.',
          ),
          ErrorHint(
            'If you are merely trying to shrink-wrap the viewport in the main axis direction, '
            'consider a RenderShrinkWrappingViewport render object (ShrinkWrappingViewport widget), '
            'which achieves that effect without implementing the intrinsic dimension API.',
          ),
        ]);
      }
      return true;
    }());
    return true;
  }

  @override
  double computeMinIntrinsicWidth(double height) {
    assert(debugThrowIfNotCheckingIntrinsics());
    return 0.0;
  }

  @override
  double computeMaxIntrinsicWidth(double height) {
    assert(debugThrowIfNotCheckingIntrinsics());
    return 0.0;
  }

  @override
  double computeMinIntrinsicHeight(double width) {
    assert(debugThrowIfNotCheckingIntrinsics());
    return 0.0;
  }

  @override
  double computeMaxIntrinsicHeight(double width) {
    assert(debugThrowIfNotCheckingIntrinsics());
    return 0.0;
  }

  @override
  bool get isRepaintBoundary => true;

  /// Determines the size and position of some of the children of the viewport.
  ///
  /// This function is the workhorse of `performLayout` implementations in
  /// subclasses.
  ///
  /// Layout starts with `child`, proceeds according to the `advance` callback,
  /// and stops once `advance` returns null.
  ///
  ///  * `scrollOffset` is the [SliverConstraints.scrollOffset] to pass the
  ///    first child. The scroll offset is adjusted by
  ///    [SliverGeometry.scrollExtent] for subsequent children.
  ///  * `overlap` is the [SliverConstraints.overlap] to pass the first child.
  ///    The overlay is adjusted by the [SliverGeometry.paintOrigin] and
  ///    [SliverGeometry.paintExtent] for subsequent children.
  ///  * `layoutOffset` is the layout offset at which to place the first child.
  ///    The layout offset is updated by the [SliverGeometry.layoutExtent] for
  ///    subsequent children.
  ///  * `remainingPaintExtent` is [SliverConstraints.remainingPaintExtent] to
  ///    pass the first child. The remaining paint extent is updated by the
  ///    [SliverGeometry.layoutExtent] for subsequent children.
  ///  * `mainAxisExtent` is the [SliverConstraints.viewportMainAxisExtent] to
  ///    pass to each child.
  ///  * `crossAxisExtent` is the [SliverConstraints.crossAxisExtent] to pass to
  ///    each child.
  ///  * `growthDirection` is the [SliverConstraints.growthDirection] to pass to
  ///    each child.
  ///
  /// Returns the first non-zero [SliverGeometry.scrollOffsetCorrection]
  /// encountered, if any. Otherwise returns 0.0. Typical callers will call this
  /// function repeatedly until it returns 0.0.
  @protected
  double layoutChildSequence({
    required RenderSliver? child,
    required double scrollOffset,
    required double overlap,
    required double layoutOffset,
    required double remainingPaintExtent,
    required double mainAxisExtent,
    required double crossAxisExtent,
    required GrowthDirection growthDirection,
    required RenderSliver? Function(RenderSliver child) advance,
    required double remainingCacheExtent,
    required double cacheOrigin,
  }) {
    assert(scrollOffset.isFinite);
    assert(scrollOffset >= 0.0);
    final double initialLayoutOffset = layoutOffset;
    final ScrollDirection adjustedUserScrollDirection = applyGrowthDirectionToScrollDirection(
      offset.userScrollDirection,
      growthDirection,
    );
    double maxPaintOffset = layoutOffset + overlap;
    double precedingScrollExtent = 0.0;

    while (child != null) {
      final double sliverScrollOffset = scrollOffset <= 0.0 ? 0.0 : scrollOffset;
      // If the scrollOffset is too small we adjust the paddedOrigin because it
      // doesn't make sense to ask a sliver for content before its scroll
      // offset.
      final double correctedCacheOrigin = math.max(cacheOrigin, -sliverScrollOffset);
      final double cacheExtentCorrection = cacheOrigin - correctedCacheOrigin;

      assert(sliverScrollOffset >= correctedCacheOrigin.abs());
      assert(correctedCacheOrigin <= 0.0);
      assert(sliverScrollOffset >= 0.0);
      assert(cacheExtentCorrection <= 0.0);

      child.layout(
        SliverConstraints(
          axisDirection: axisDirection,
          growthDirection: growthDirection,
          userScrollDirection: adjustedUserScrollDirection,
          scrollOffset: sliverScrollOffset,
          precedingScrollExtent: precedingScrollExtent,
          overlap: maxPaintOffset - layoutOffset,
          remainingPaintExtent: math.max(
            0.0,
            remainingPaintExtent - layoutOffset + initialLayoutOffset,
          ),
          crossAxisExtent: crossAxisExtent,
          crossAxisDirection: crossAxisDirection,
          viewportMainAxisExtent: mainAxisExtent,
          remainingCacheExtent: math.max(0.0, remainingCacheExtent + cacheExtentCorrection),
          cacheOrigin: correctedCacheOrigin,
        ),
        parentUsesSize: true,
      );

      final SliverGeometry childLayoutGeometry = child.geometry!;
      assert(childLayoutGeometry.debugAssertIsValid());

      // If there is a correction to apply, we'll have to start over.
      if (childLayoutGeometry.scrollOffsetCorrection != null) {
        return childLayoutGeometry.scrollOffsetCorrection!;
      }

      // We use the child's paint origin in our coordinate system as the
      // layoutOffset we store in the child's parent data.
      final double effectiveLayoutOffset = layoutOffset + childLayoutGeometry.paintOrigin;

      // `effectiveLayoutOffset` becomes meaningless once we moved past the trailing edge
      // because `childLayoutGeometry.layoutExtent` is zero. Using the still increasing
      // 'scrollOffset` to roughly position these invisible slivers in the right order.
      if (childLayoutGeometry.visible || scrollOffset > 0) {
        updateChildLayoutOffset(child, effectiveLayoutOffset, growthDirection);
      } else {
        updateChildLayoutOffset(child, -scrollOffset + initialLayoutOffset, growthDirection);
      }

      maxPaintOffset = math.max(
        effectiveLayoutOffset + childLayoutGeometry.paintExtent,
        maxPaintOffset,
      );
      scrollOffset -= childLayoutGeometry.scrollExtent;
      precedingScrollExtent += childLayoutGeometry.scrollExtent;
      layoutOffset += childLayoutGeometry.layoutExtent;
      if (childLayoutGeometry.cacheExtent != 0.0) {
        remainingCacheExtent -= childLayoutGeometry.cacheExtent - cacheExtentCorrection;
        cacheOrigin = math.min(correctedCacheOrigin + childLayoutGeometry.cacheExtent, 0.0);
      }

      updateOutOfBandData(growthDirection, childLayoutGeometry);

      // move on to the next child
      child = advance(child);
    }

    // we made it without a correction, whee!
    return 0.0;
  }

  @override
  Rect? describeApproximatePaintClip(RenderSliver child) {
    switch (clipBehavior) {
      case Clip.none:
        return null;
      case Clip.hardEdge:
      case Clip.antiAlias:
      case Clip.antiAliasWithSaveLayer:
        break;
    }

    final Rect viewportClip = Offset.zero & size;
    // The child's viewportMainAxisExtent can be infinite when a
    // RenderShrinkWrappingViewport is given infinite constraints, such as when
    // it is the child of a Row or Column (depending on orientation).
    //
    // For example, a shrink wrapping render sliver may have infinite
    // constraints along the viewport's main axis but may also have bouncing
    // scroll physics, which will allow for some scrolling effect to occur.
    // We should just use the viewportClip - the start of the overlap is at
    // double.infinity and so it is effectively meaningless.
    if (child.constraints.overlap == 0 || !child.constraints.viewportMainAxisExtent.isFinite) {
      return viewportClip;
    }

    // Adjust the clip rect for this sliver by the overlap from the previous sliver.
    double left = viewportClip.left;
    double right = viewportClip.right;
    double top = viewportClip.top;
    double bottom = viewportClip.bottom;
    final double startOfOverlap =
        child.constraints.viewportMainAxisExtent - child.constraints.remainingPaintExtent;
    final double overlapCorrection = startOfOverlap + child.constraints.overlap;
    switch (applyGrowthDirectionToAxisDirection(axisDirection, child.constraints.growthDirection)) {
      case AxisDirection.down:
        top += overlapCorrection;
      case AxisDirection.up:
        bottom -= overlapCorrection;
      case AxisDirection.right:
        left += overlapCorrection;
      case AxisDirection.left:
        right -= overlapCorrection;
    }
    return Rect.fromLTRB(left, top, right, bottom);
  }

  @override
  Rect describeSemanticsClip(RenderSliver? child) {
    if (_calculatedCacheExtent == null) {
      return semanticBounds;
    }

    switch (axis) {
      case Axis.vertical:
        return Rect.fromLTRB(
          semanticBounds.left,
          semanticBounds.top - _calculatedCacheExtent!,
          semanticBounds.right,
          semanticBounds.bottom + _calculatedCacheExtent!,
        );
      case Axis.horizontal:
        return Rect.fromLTRB(
          semanticBounds.left - _calculatedCacheExtent!,
          semanticBounds.top,
          semanticBounds.right + _calculatedCacheExtent!,
          semanticBounds.bottom,
        );
    }
  }

  @override
  void paint(PaintingContext context, Offset offset) {
    if (firstChild == null) {
      return;
    }
    if (hasVisualOverflow && clipBehavior != Clip.none) {
      _clipRectLayer.layer = context.pushClipRect(
        needsCompositing,
        offset,
        Offset.zero & size,
        _paintContents,
        clipBehavior: clipBehavior,
        oldLayer: _clipRectLayer.layer,
      );
    } else {
      _clipRectLayer.layer = null;
      _paintContents(context, offset);
    }
  }

  final LayerHandle<ClipRectLayer> _clipRectLayer = LayerHandle<ClipRectLayer>();

  @override
  void dispose() {
    _clipRectLayer.layer = null;
    super.dispose();
  }

  void _paintContents(PaintingContext context, Offset offset) {
    for (final RenderSliver child in childrenInPaintOrder) {
      if (child.geometry!.visible) {
        context.paintChild(child, offset + paintOffsetOf(child));
      }
    }
  }

  @override
  void debugPaintSize(PaintingContext context, Offset offset) {
    assert(() {
      super.debugPaintSize(context, offset);
      final Paint paint =
          Paint()
            ..style = PaintingStyle.stroke
            ..strokeWidth = 1.0
            ..color = const Color(0xFF00FF00);
      final Canvas canvas = context.canvas;
      RenderSliver? child = firstChild;
      while (child != null) {
        final Size size = switch (axis) {
          Axis.vertical => Size(child.constraints.crossAxisExtent, child.geometry!.layoutExtent),
          Axis.horizontal => Size(child.geometry!.layoutExtent, child.constraints.crossAxisExtent),
        };
        canvas.drawRect(((offset + paintOffsetOf(child)) & size).deflate(0.5), paint);
        child = childAfter(child);
      }
      return true;
    }());
  }

  @override
  bool hitTestChildren(BoxHitTestResult result, {required Offset position}) {
    final (double mainAxisPosition, double crossAxisPosition) = switch (axis) {
      Axis.vertical => (position.dy, position.dx),
      Axis.horizontal => (position.dx, position.dy),
    };
    final SliverHitTestResult sliverResult = SliverHitTestResult.wrap(result);
    for (final RenderSliver child in childrenInHitTestOrder) {
      if (!child.geometry!.visible) {
        continue;
      }
      final Matrix4 transform = Matrix4.identity();
      applyPaintTransform(child, transform); // must be invertible
      final bool isHit = result.addWithOutOfBandPosition(
        paintTransform: transform,
        hitTest: (BoxHitTestResult result) {
          return child.hitTest(
            sliverResult,
            mainAxisPosition: computeChildMainAxisPosition(child, mainAxisPosition),
            crossAxisPosition: crossAxisPosition,
          );
        },
      );
      if (isHit) {
        return true;
      }
    }
    return false;
  }

  @override
  RevealedOffset getOffsetToReveal(
    RenderObject target,
    double alignment, {
    Rect? rect,
    Axis? axis,
  }) {
    // One dimensional viewport has only one axis, override if it was
    // provided/may be mismatched.
    axis = this.axis;

    // Steps to convert `rect` (from a RenderBox coordinate system) to its
    // scroll offset within this viewport (not in the exact order):
    //
    // 1. Pick the outermost RenderBox (between which, and the viewport, there
    // is nothing but RenderSlivers) as an intermediate reference frame
    // (the `pivot`), convert `rect` to that coordinate space.
    //
    // 2. Convert `rect` from the `pivot` coordinate space to its sliver
    // parent's sliver coordinate system (i.e., to a scroll offset), based on
    // the axis direction and growth direction of the parent.
    //
    // 3. Convert the scroll offset to its sliver parent's coordinate space
    // using `childScrollOffset`, until we reach the viewport.
    //
    // 4. Make the final conversion from the outmost sliver to the viewport
    // using `scrollOffsetOf`.

    double leadingScrollOffset = 0.0;
    // Starting at `target` and walking towards the root:
    //  - `child` will be the last object before we reach this viewport, and
    //  - `pivot` will be the last RenderBox before we reach this viewport.
    RenderObject child = target;
    RenderBox? pivot;
    bool onlySlivers =
        target is RenderSliver; // ... between viewport and `target` (`target` included).
    while (child.parent != this) {
      final RenderObject parent = child.parent!;
      if (child is RenderBox) {
        pivot = child;
      }
      if (parent is RenderSliver) {
        leadingScrollOffset += parent.childScrollOffset(child)!;
      } else {
        onlySlivers = false;
        leadingScrollOffset = 0.0;
      }
      child = parent;
    }

    // `rect` in the new intermediate coordinate system.
    final Rect rectLocal;
    // Our new reference frame render object's main axis extent.
    final double pivotExtent;
    final GrowthDirection growthDirection;

    // `leadingScrollOffset` is currently the scrollOffset of our new reference
    // frame (`pivot` or `target`), within `child`.
    if (pivot != null) {
      assert(pivot.parent != null);
      assert(pivot.parent != this);
      assert(pivot != this);
      assert(
        pivot.parent is RenderSliver,
      ); // TODO(abarth): Support other kinds of render objects besides slivers.
      final RenderSliver pivotParent = pivot.parent! as RenderSliver;
      growthDirection = pivotParent.constraints.growthDirection;
      pivotExtent = switch (axis) {
        Axis.horizontal => pivot.size.width,
        Axis.vertical => pivot.size.height,
      };
      rect ??= target.paintBounds;
      rectLocal = MatrixUtils.transformRect(target.getTransformTo(pivot), rect);
    } else if (onlySlivers) {
      // `pivot` does not exist. We'll have to make up one from `target`, the
      // innermost sliver.
      final RenderSliver targetSliver = target as RenderSliver;
      growthDirection = targetSliver.constraints.growthDirection;
      // TODO(LongCatIsLooong): make sure this works if `targetSliver` is a
      // persistent header, when #56413 relands.
      pivotExtent = targetSliver.geometry!.scrollExtent;
      if (rect == null) {
        switch (axis) {
          case Axis.horizontal:
            rect = Rect.fromLTWH(
              0,
              0,
              targetSliver.geometry!.scrollExtent,
              targetSliver.constraints.crossAxisExtent,
            );
          case Axis.vertical:
            rect = Rect.fromLTWH(
              0,
              0,
              targetSliver.constraints.crossAxisExtent,
              targetSliver.geometry!.scrollExtent,
            );
        }
      }
      rectLocal = rect;
    } else {
      assert(rect != null);
      return RevealedOffset(offset: offset.pixels, rect: rect!);
    }

    assert(child.parent == this);
    assert(child is RenderSliver);
    final RenderSliver sliver = child as RenderSliver;

    // The scroll offset of `rect` within `child`.
    leadingScrollOffset += switch (applyGrowthDirectionToAxisDirection(
      axisDirection,
      growthDirection,
    )) {
      AxisDirection.up => pivotExtent - rectLocal.bottom,
      AxisDirection.left => pivotExtent - rectLocal.right,
      AxisDirection.right => rectLocal.left,
      AxisDirection.down => rectLocal.top,
    };

    // So far leadingScrollOffset is the scroll offset of `rect` in the `child`
    // sliver's sliver coordinate system. The sign of this value indicates
    // whether the `rect` protrudes the leading edge of the `child` sliver. When
    // this value is non-negative and `child`'s `maxScrollObstructionExtent` is
    // greater than 0, we assume `rect` can't be obstructed by the leading edge
    // of the viewport (i.e. its pinned to the leading edge).
    final bool isPinned =
        sliver.geometry!.maxScrollObstructionExtent > 0 && leadingScrollOffset >= 0;

    // The scroll offset in the viewport to `rect`.
    leadingScrollOffset = scrollOffsetOf(sliver, leadingScrollOffset);

    // This step assumes the viewport's layout is up-to-date, i.e., if
    // offset.pixels is changed after the last performLayout, the new scroll
    // position will not be accounted for.
    final Matrix4 transform = target.getTransformTo(this);
    Rect targetRect = MatrixUtils.transformRect(transform, rect);
    final double extentOfPinnedSlivers = maxScrollObstructionExtentBefore(sliver);

    switch (sliver.constraints.growthDirection) {
      case GrowthDirection.forward:
        if (isPinned && alignment <= 0) {
          return RevealedOffset(offset: double.infinity, rect: targetRect);
        }
        leadingScrollOffset -= extentOfPinnedSlivers;
      case GrowthDirection.reverse:
        if (isPinned && alignment >= 1) {
          return RevealedOffset(offset: double.negativeInfinity, rect: targetRect);
        }
        // If child's growth direction is reverse, when viewport.offset is
        // `leadingScrollOffset`, it is positioned just outside of the leading
        // edge of the viewport.
        leadingScrollOffset -= switch (axis) {
          Axis.vertical => targetRect.height,
          Axis.horizontal => targetRect.width,
        };
    }

    final double mainAxisExtentDifference = switch (axis) {
      Axis.horizontal => size.width - extentOfPinnedSlivers - rectLocal.width,
      Axis.vertical => size.height - extentOfPinnedSlivers - rectLocal.height,
    };

    final double targetOffset = leadingScrollOffset - mainAxisExtentDifference * alignment;
    final double offsetDifference = offset.pixels - targetOffset;

    targetRect = switch (axisDirection) {
      AxisDirection.up => targetRect.translate(0.0, -offsetDifference),
      AxisDirection.down => targetRect.translate(0.0, offsetDifference),
      AxisDirection.left => targetRect.translate(-offsetDifference, 0.0),
      AxisDirection.right => targetRect.translate(offsetDifference, 0.0),
    };

    return RevealedOffset(offset: targetOffset, rect: targetRect);
  }

  /// The offset at which the given `child` should be painted.
  ///
  /// The returned offset is from the top left corner of the inside of the
  /// viewport to the top left corner of the paint coordinate system of the
  /// `child`.
  ///
  /// See also:
  ///
  ///  * [paintOffsetOf], which uses the layout offset and growth direction
  ///    computed for the child during layout.
  @protected
  Offset computeAbsolutePaintOffset(
    RenderSliver child,
    double layoutOffset,
    GrowthDirection growthDirection,
  ) {
    assert(hasSize); // this is only usable once we have a size
    assert(child.geometry != null);
    return switch (applyGrowthDirectionToAxisDirection(axisDirection, growthDirection)) {
      AxisDirection.up => Offset(0.0, size.height - layoutOffset - child.geometry!.paintExtent),
      AxisDirection.left => Offset(size.width - layoutOffset - child.geometry!.paintExtent, 0.0),
      AxisDirection.right => Offset(layoutOffset, 0.0),
      AxisDirection.down => Offset(0.0, layoutOffset),
    };
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(EnumProperty<AxisDirection>('axisDirection', axisDirection));
    properties.add(EnumProperty<AxisDirection>('crossAxisDirection', crossAxisDirection));
    properties.add(DiagnosticsProperty<ViewportOffset>('offset', offset));
  }

  @override
  List<DiagnosticsNode> debugDescribeChildren() {
    final List<DiagnosticsNode> children = <DiagnosticsNode>[];
    RenderSliver? child = firstChild;
    if (child == null) {
      return children;
    }

    int count = indexOfFirstChild;
    while (true) {
      children.add(child!.toDiagnosticsNode(name: labelForChild(count)));
      if (child == lastChild) {
        break;
      }
      count += 1;
      child = childAfter(child);
    }
    return children;
  }

  // API TO BE IMPLEMENTED BY SUBCLASSES

  // setupParentData

  // performLayout (and optionally sizedByParent and performResize)

  /// Whether the contents of this viewport would paint outside the bounds of
  /// the viewport if [paint] did not clip.
  ///
  /// This property enables an optimization whereby [paint] can skip apply a
  /// clip of the contents of the viewport are known to paint entirely within
  /// the bounds of the viewport.
  @protected
  bool get hasVisualOverflow;

  /// Called during [layoutChildSequence] for each child.
  ///
  /// Typically used by subclasses to update any out-of-band data, such as the
  /// max scroll extent, for each child.
  @protected
  void updateOutOfBandData(GrowthDirection growthDirection, SliverGeometry childLayoutGeometry);

  /// Called during [layoutChildSequence] to store the layout offset for the
  /// given child.
  ///
  /// Different subclasses using different representations for their children's
  /// layout offset (e.g., logical or physical coordinates). This function lets
  /// subclasses transform the child's layout offset before storing it in the
  /// child's parent data.
  @protected
  void updateChildLayoutOffset(
    RenderSliver child,
    double layoutOffset,
    GrowthDirection growthDirection,
  );

  /// The offset at which the given `child` should be painted.
  ///
  /// The returned offset is from the top left corner of the inside of the
  /// viewport to the top left corner of the paint coordinate system of the
  /// `child`.
  ///
  /// See also:
  ///
  ///  * [computeAbsolutePaintOffset], which computes the paint offset from an
  ///    explicit layout offset and growth direction instead of using the values
  ///    computed for the child during layout.
  @protected
  Offset paintOffsetOf(RenderSliver child);

  /// Returns the scroll offset within the viewport for the given
  /// `scrollOffsetWithinChild` within the given `child`.
  ///
  /// The returned value is an estimate that assumes the slivers within the
  /// viewport do not change the layout extent in response to changes in their
  /// scroll offset.
  @protected
  double scrollOffsetOf(RenderSliver child, double scrollOffsetWithinChild);

  /// Returns the total scroll obstruction extent of all slivers in the viewport
  /// before [child].
  ///
  /// This is the extent by which the actual area in which content can scroll
  /// is reduced. For example, an app bar that is pinned at the top will reduce
  /// the area in which content can actually scroll by the height of the app bar.
  @protected
  double maxScrollObstructionExtentBefore(RenderSliver child);

  /// Converts the `parentMainAxisPosition` into the child's coordinate system.
  ///
  /// The `parentMainAxisPosition` is a distance from the top edge (for vertical
  /// viewports) or left edge (for horizontal viewports) of the viewport bounds.
  /// This describes a line, perpendicular to the viewport's main axis, heretofore
  /// known as the target line.
  ///
  /// The child's coordinate system's origin in the main axis is at the leading
  /// edge of the given child, as given by the child's
  /// [SliverConstraints.axisDirection] and [SliverConstraints.growthDirection].
  ///
  /// This method returns the distance from the leading edge of the given child to
  /// the target line described above.
  ///
  /// (The `parentMainAxisPosition` is not from the leading edge of the
  /// viewport, it's always the top or left edge.)
  @protected
  double computeChildMainAxisPosition(RenderSliver child, double parentMainAxisPosition);

  /// The index of the first child of the viewport relative to the center child.
  ///
  /// For example, the center child has index zero and the first child in the
  /// reverse growth direction has index -1.
  @protected
  int get indexOfFirstChild;

  /// A short string to identify the child with the given index.
  ///
  /// Used by [debugDescribeChildren] to label the children.
  @protected
  String labelForChild(int index);

  /// Provides an iterable that walks the children of the viewport, in the order
  /// that they should be painted.
  ///
  /// This should be the reverse order of [childrenInHitTestOrder].
  @protected
  Iterable<RenderSliver> get childrenInPaintOrder;

  /// Provides an iterable that walks the children of the viewport, in the order
  /// that hit-testing should use.
  ///
  /// This should be the reverse order of [childrenInPaintOrder].
  @protected
  Iterable<RenderSliver> get childrenInHitTestOrder;

  @override
  void showOnScreen({
    RenderObject? descendant,
    Rect? rect,
    Duration duration = Duration.zero,
    Curve curve = Curves.ease,
  }) {
    if (!offset.allowImplicitScrolling) {
      return super.showOnScreen(
        descendant: descendant,
        rect: rect,
        duration: duration,
        curve: curve,
      );
    }

    final Rect? newRect = RenderViewportBase.showInViewport(
      descendant: descendant,
      viewport: this,
      offset: offset,
      rect: rect,
      duration: duration,
      curve: curve,
    );
    super.showOnScreen(rect: newRect, duration: duration, curve: curve);
  }

  /// Make (a portion of) the given `descendant` of the given `viewport` fully
  /// visible in the `viewport` by manipulating the provided [ViewportOffset]
  /// `offset`.
  ///
  /// The optional `rect` parameter describes which area of the `descendant`
  /// should be shown in the viewport. If `rect` is null, the entire
  /// `descendant` will be revealed. The `rect` parameter is interpreted
  /// relative to the coordinate system of `descendant`.
  ///
  /// The returned [Rect] describes the new location of `descendant` or `rect`
  /// in the viewport after it has been revealed. See [RevealedOffset.rect]
  /// for a full definition of this [Rect].
  ///
  /// If `descendant` is null, this is a no-op and `rect` is returned.
  ///
  /// If both `descendant` and `rect` are null, null is returned because there is
  /// nothing to be shown in the viewport.
  ///
  /// The `duration` parameter can be set to a non-zero value to animate the
  /// target object into the viewport with an animation defined by `curve`.
  ///
  /// See also:
  ///
  /// * [RenderObject.showOnScreen], overridden by [RenderViewportBase] and the
  ///   renderer for [SingleChildScrollView] to delegate to this method.
  static Rect? showInViewport({
    RenderObject? descendant,
    Rect? rect,
    required RenderAbstractViewport viewport,
    required ViewportOffset offset,
    Duration duration = Duration.zero,
    Curve curve = Curves.ease,
  }) {
    if (descendant == null) {
      return rect;
    }
    final RevealedOffset leadingEdgeOffset = viewport.getOffsetToReveal(
      descendant,
      0.0,
      rect: rect,
    );
    final RevealedOffset trailingEdgeOffset = viewport.getOffsetToReveal(
      descendant,
      1.0,
      rect: rect,
    );
    final double currentOffset = offset.pixels;
    final RevealedOffset? targetOffset = RevealedOffset.clampOffset(
      leadingEdgeOffset: leadingEdgeOffset,
      trailingEdgeOffset: trailingEdgeOffset,
      currentOffset: currentOffset,
    );
    if (targetOffset == null) {
      // `descendant` is between leading and trailing edge and hence already
      //  fully shown on screen. No action necessary.
      assert(viewport.parent != null);
      final Matrix4 transform = descendant.getTransformTo(viewport.parent);
      return MatrixUtils.transformRect(transform, rect ?? descendant.paintBounds);
    }

    offset.moveTo(targetOffset.offset, duration: duration, curve: curve);
    return targetOffset.rect;
  }
}

/// A render object that is bigger on the inside.
///
/// [RenderViewport] is the visual workhorse of the scrolling machinery. It
/// displays a subset of its children according to its own dimensions and the
/// given [offset]. As the offset varies, different children are visible through
/// the viewport.
///
/// [RenderViewport] hosts a bidirectional list of slivers in a single shared
/// [Axis], anchored on a [center] sliver, which is placed at the zero scroll
/// offset. The center widget is displayed in the viewport according to the
/// [anchor] property.
///
/// Slivers that are earlier in the child list than [center] are displayed in
/// reverse order in the reverse [axisDirection] starting from the [center]. For
/// example, if the [axisDirection] is [AxisDirection.down], the first sliver
/// before [center] is placed above the [center]. The slivers that are later in
/// the child list than [center] are placed in order in the [axisDirection]. For
/// example, in the preceding scenario, the first sliver after [center] is
/// placed below the [center].
///
/// {@macro flutter.rendering.GrowthDirection.sample}
///
/// [RenderViewport] cannot contain [RenderBox] children directly. Instead, use
/// a [RenderSliverList], [RenderSliverFixedExtentList], [RenderSliverGrid], or
/// a [RenderSliverToBoxAdapter], for example.
///
/// See also:
///
///  * [RenderSliver], which explains more about the Sliver protocol.
///  * [RenderBox], which explains more about the Box protocol.
///  * [RenderSliverToBoxAdapter], which allows a [RenderBox] object to be
///    placed inside a [RenderSliver] (the opposite of this class).
///  * [RenderShrinkWrappingViewport], a variant of [RenderViewport] that
///    shrink-wraps its contents along the main axis.
class RenderViewport extends RenderViewportBase<SliverPhysicalContainerParentData> {
  /// Creates a viewport for [RenderSliver] objects.
  ///
  /// If the [center] is not specified, then the first child in the `children`
  /// list, if any, is used.
  ///
  /// The [offset] must be specified. For testing purposes, consider passing a
  /// [ViewportOffset.zero] or [ViewportOffset.fixed].
  RenderViewport({
    super.axisDirection,
    required super.crossAxisDirection,
    required super.offset,
    double anchor = 0.0,
    List<RenderSliver>? children,
    RenderSliver? center,
    super.cacheExtent,
    super.cacheExtentStyle,
    super.clipBehavior,
  }) : assert(anchor >= 0.0 && anchor <= 1.0),
       assert(cacheExtentStyle != CacheExtentStyle.viewport || cacheExtent != null),
       _anchor = anchor,
       _center = center {
    addAll(children);
    if (center == null && firstChild != null) {
      _center = firstChild;
    }
  }

  /// If a [RenderAbstractViewport] overrides
  /// [RenderObject.describeSemanticsConfiguration] to add the [SemanticsTag]
  /// [useTwoPaneSemantics] to its [SemanticsConfiguration], two semantics nodes
  /// will be used to represent the viewport with its associated scrolling
  /// actions in the semantics tree.
  ///
  /// Two semantics nodes (an inner and an outer node) are necessary to exclude
  /// certain child nodes (via the [excludeFromScrolling] tag) from the
  /// scrollable area for semantic purposes: The [SemanticsNode]s of children
  /// that should be excluded from scrolling will be attached to the outer node.
  /// The semantic scrolling actions and the [SemanticsNode]s of scrollable
  /// children will be attached to the inner node, which itself is a child of
  /// the outer node.
  ///
  /// See also:
  ///
  /// * [RenderViewportBase.describeSemanticsConfiguration], which adds this
  ///   tag to its [SemanticsConfiguration].
  static const SemanticsTag useTwoPaneSemantics = SemanticsTag('RenderViewport.twoPane');

  /// When a top-level [SemanticsNode] below a [RenderAbstractViewport] is
  /// tagged with [excludeFromScrolling] it will not be part of the scrolling
  /// area for semantic purposes.
  ///
  /// This behavior is only active if the [RenderAbstractViewport]
  /// tagged its [SemanticsConfiguration] with [useTwoPaneSemantics].
  /// Otherwise, the [excludeFromScrolling] tag is ignored.
  ///
  /// As an example, a [RenderSliver] that stays on the screen within a
  /// [Scrollable] even though the user has scrolled past it (e.g. a pinned app
  /// bar) can tag its [SemanticsNode] with [excludeFromScrolling] to indicate
  /// that it should no longer be considered for semantic actions related to
  /// scrolling.
  static const SemanticsTag excludeFromScrolling = SemanticsTag(
    'RenderViewport.excludeFromScrolling',
  );

  @override
  void setupParentData(RenderObject child) {
    if (child.parentData is! SliverPhysicalContainerParentData) {
      child.parentData = SliverPhysicalContainerParentData();
    }
  }

  /// The relative position of the zero scroll offset.
  ///
  /// For example, if [anchor] is 0.5 and the [axisDirection] is
  /// [AxisDirection.down] or [AxisDirection.up], then the zero scroll offset is
  /// vertically centered within the viewport. If the [anchor] is 1.0, and the
  /// [axisDirection] is [AxisDirection.right], then the zero scroll offset is
  /// on the left edge of the viewport.
  ///
  /// {@macro flutter.rendering.GrowthDirection.sample}
  double get anchor => _anchor;
  double _anchor;
  set anchor(double value) {
    assert(value >= 0.0 && value <= 1.0);
    if (value == _anchor) {
      return;
    }
    _anchor = value;
    markNeedsLayout();
  }

  /// The first child in the [GrowthDirection.forward] growth direction.
  ///
  /// This child that will be at the position defined by [anchor] when the
  /// [ViewportOffset.pixels] of [offset] is `0`.
  ///
  /// Children after [center] will be placed in the [axisDirection] relative to
  /// the [center].
  ///
  /// Children before [center] will be placed in the opposite of
  /// the [axisDirection] relative to the [center]. These children above
  /// [center] will have a growth direction of [GrowthDirection.reverse].
  ///
  /// The [center] must be a direct child of the viewport.
  ///
  /// {@macro flutter.rendering.GrowthDirection.sample}
  RenderSliver? get center => _center;
  RenderSliver? _center;
  set center(RenderSliver? value) {
    if (value == _center) {
      return;
    }
    _center = value;
    markNeedsLayout();
  }

  @override
  bool get sizedByParent => true;

  @override
  @protected
  Size computeDryLayout(covariant BoxConstraints constraints) {
    assert(debugCheckHasBoundedAxis(axis, constraints));
    return constraints.biggest;
  }

  static const int _maxLayoutCyclesPerChild = 10;

  // Out-of-band data computed during layout.
  late double _minScrollExtent;
  late double _maxScrollExtent;
  bool _hasVisualOverflow = false;

  @override
  void performLayout() {
    // Ignore the return value of applyViewportDimension because we are
    // doing a layout regardless.
    switch (axis) {
      case Axis.vertical:
        offset.applyViewportDimension(size.height);
      case Axis.horizontal:
        offset.applyViewportDimension(size.width);
    }

    if (center == null) {
      assert(firstChild == null);
      _minScrollExtent = 0.0;
      _maxScrollExtent = 0.0;
      _hasVisualOverflow = false;
      offset.applyContentDimensions(0.0, 0.0);
      return;
    }
    assert(center!.parent == this);

    final (double mainAxisExtent, double crossAxisExtent) = switch (axis) {
      Axis.vertical => (size.height, size.width),
      Axis.horizontal => (size.width, size.height),
    };

    final double centerOffsetAdjustment = center!.centerOffsetAdjustment;
    final int maxLayoutCycles = _maxLayoutCyclesPerChild * childCount;

    double correction;
    int count = 0;
    do {
      correction = _attemptLayout(
        mainAxisExtent,
        crossAxisExtent,
        offset.pixels + centerOffsetAdjustment,
      );
      if (correction != 0.0) {
        offset.correctBy(correction);
      } else {
        if (offset.applyContentDimensions(
          math.min(0.0, _minScrollExtent + mainAxisExtent * anchor),
          math.max(0.0, _maxScrollExtent - mainAxisExtent * (1.0 - anchor)),
        )) {
          break;
        }
      }
      count += 1;
    } while (count < maxLayoutCycles);
    assert(() {
      if (count >= maxLayoutCycles) {
        assert(count != 1);
        throw FlutterError(
          'A RenderViewport exceeded its maximum number of layout cycles.\n'
          'RenderViewport render objects, during layout, can retry if either their '
          'slivers or their ViewportOffset decide that the offset should be corrected '
          'to take into account information collected during that layout.\n'
          'In the case of this RenderViewport object, however, this happened $count '
          'times and still there was no consensus on the scroll offset. This usually '
          'indicates a bug. Specifically, it means that one of the following three '
          'problems is being experienced by the RenderViewport object:\n'
          ' * One of the RenderSliver children or the ViewportOffset have a bug such'
          ' that they always think that they need to correct the offset regardless.\n'
          ' * Some combination of the RenderSliver children and the ViewportOffset'
          ' have a bad interaction such that one applies a correction then another'
          ' applies a reverse correction, leading to an infinite loop of corrections.\n'
          ' * There is a pathological case that would eventually resolve, but it is'
          ' so complicated that it cannot be resolved in any reasonable number of'
          ' layout passes.',
        );
      }
      return true;
    }());
  }

  double _attemptLayout(double mainAxisExtent, double crossAxisExtent, double correctedOffset) {
    assert(!mainAxisExtent.isNaN);
    assert(mainAxisExtent >= 0.0);
    assert(crossAxisExtent.isFinite);
    assert(crossAxisExtent >= 0.0);
    assert(correctedOffset.isFinite);
    _minScrollExtent = 0.0;
    _maxScrollExtent = 0.0;
    _hasVisualOverflow = false;

    // centerOffset is the offset from the leading edge of the RenderViewport
    // to the zero scroll offset (the line between the forward slivers and the
    // reverse slivers).
    final double centerOffset = mainAxisExtent * anchor - correctedOffset;
    final double reverseDirectionRemainingPaintExtent = clampDouble(
      centerOffset,
      0.0,
      mainAxisExtent,
    );
    final double forwardDirectionRemainingPaintExtent = clampDouble(
      mainAxisExtent - centerOffset,
      0.0,
      mainAxisExtent,
    );

    _calculatedCacheExtent = switch (cacheExtentStyle) {
      CacheExtentStyle.pixel => cacheExtent,
      CacheExtentStyle.viewport => mainAxisExtent * _cacheExtent,
    };

    final double fullCacheExtent = mainAxisExtent + 2 * _calculatedCacheExtent!;
    final double centerCacheOffset = centerOffset + _calculatedCacheExtent!;
    final double reverseDirectionRemainingCacheExtent = clampDouble(
      centerCacheOffset,
      0.0,
      fullCacheExtent,
    );
    final double forwardDirectionRemainingCacheExtent = clampDouble(
      fullCacheExtent - centerCacheOffset,
      0.0,
      fullCacheExtent,
    );

    final RenderSliver? leadingNegativeChild = childBefore(center!);

    if (leadingNegativeChild != null) {
      // negative scroll offsets
      final double result = layoutChildSequence(
        child: leadingNegativeChild,
        scrollOffset: math.max(mainAxisExtent, centerOffset) - mainAxisExtent,
        overlap: 0.0,
        layoutOffset: forwardDirectionRemainingPaintExtent,
        remainingPaintExtent: reverseDirectionRemainingPaintExtent,
        mainAxisExtent: mainAxisExtent,
        crossAxisExtent: crossAxisExtent,
        growthDirection: GrowthDirection.reverse,
        advance: childBefore,
        remainingCacheExtent: reverseDirectionRemainingCacheExtent,
        cacheOrigin: clampDouble(mainAxisExtent - centerOffset, -_calculatedCacheExtent!, 0.0),
      );
      if (result != 0.0) {
        return -result;
      }
    }

    // positive scroll offsets
    return layoutChildSequence(
      child: center,
      scrollOffset: math.max(0.0, -centerOffset),
      overlap: leadingNegativeChild == null ? math.min(0.0, -centerOffset) : 0.0,
      layoutOffset:
          centerOffset >= mainAxisExtent ? centerOffset : reverseDirectionRemainingPaintExtent,
      remainingPaintExtent: forwardDirectionRemainingPaintExtent,
      mainAxisExtent: mainAxisExtent,
      crossAxisExtent: crossAxisExtent,
      growthDirection: GrowthDirection.forward,
      advance: childAfter,
      remainingCacheExtent: forwardDirectionRemainingCacheExtent,
      cacheOrigin: clampDouble(centerOffset, -_calculatedCacheExtent!, 0.0),
    );
  }

  @override
  bool get hasVisualOverflow => _hasVisualOverflow;

  @override
  void updateOutOfBandData(GrowthDirection growthDirection, SliverGeometry childLayoutGeometry) {
    switch (growthDirection) {
      case GrowthDirection.forward:
        _maxScrollExtent += childLayoutGeometry.scrollExtent;
      case GrowthDirection.reverse:
        _minScrollExtent -= childLayoutGeometry.scrollExtent;
    }
    if (childLayoutGeometry.hasVisualOverflow) {
      _hasVisualOverflow = true;
    }
  }

  @override
  void updateChildLayoutOffset(
    RenderSliver child,
    double layoutOffset,
    GrowthDirection growthDirection,
  ) {
    final SliverPhysicalParentData childParentData = child.parentData! as SliverPhysicalParentData;
    childParentData.paintOffset = computeAbsolutePaintOffset(child, layoutOffset, growthDirection);
  }

  @override
  Offset paintOffsetOf(RenderSliver child) {
    final SliverPhysicalParentData childParentData = child.parentData! as SliverPhysicalParentData;
    return childParentData.paintOffset;
  }

  @override
  double scrollOffsetOf(RenderSliver child, double scrollOffsetWithinChild) {
    assert(child.parent == this);
    final GrowthDirection growthDirection = child.constraints.growthDirection;
    switch (growthDirection) {
      case GrowthDirection.forward:
        double scrollOffsetToChild = 0.0;
        RenderSliver? current = center;
        while (current != child) {
          scrollOffsetToChild += current!.geometry!.scrollExtent;
          current = childAfter(current);
        }
        return scrollOffsetToChild + scrollOffsetWithinChild;
      case GrowthDirection.reverse:
        double scrollOffsetToChild = 0.0;
        RenderSliver? current = childBefore(center!);
        while (current != child) {
          scrollOffsetToChild -= current!.geometry!.scrollExtent;
          current = childBefore(current);
        }
        return scrollOffsetToChild - scrollOffsetWithinChild;
    }
  }

  @override
  double maxScrollObstructionExtentBefore(RenderSliver child) {
    assert(child.parent == this);
    final GrowthDirection growthDirection = child.constraints.growthDirection;
    switch (growthDirection) {
      case GrowthDirection.forward:
        double pinnedExtent = 0.0;
        RenderSliver? current = center;
        while (current != child) {
          pinnedExtent += current!.geometry!.maxScrollObstructionExtent;
          current = childAfter(current);
        }
        return pinnedExtent;
      case GrowthDirection.reverse:
        double pinnedExtent = 0.0;
        RenderSliver? current = childBefore(center!);
        while (current != child) {
          pinnedExtent += current!.geometry!.maxScrollObstructionExtent;
          current = childBefore(current);
        }
        return pinnedExtent;
    }
  }

  @override
  void applyPaintTransform(RenderObject child, Matrix4 transform) {
    // Hit test logic relies on this always providing an invertible matrix.
    final SliverPhysicalParentData childParentData = child.parentData! as SliverPhysicalParentData;
    childParentData.applyPaintTransform(transform);
  }

  @override
  double computeChildMainAxisPosition(RenderSliver child, double parentMainAxisPosition) {
    final Offset paintOffset = (child.parentData! as SliverPhysicalParentData).paintOffset;
    return switch (applyGrowthDirectionToAxisDirection(
      child.constraints.axisDirection,
      child.constraints.growthDirection,
    )) {
      AxisDirection.down => parentMainAxisPosition - paintOffset.dy,
      AxisDirection.right => parentMainAxisPosition - paintOffset.dx,
      AxisDirection.up => child.geometry!.paintExtent - (parentMainAxisPosition - paintOffset.dy),
      AxisDirection.left => child.geometry!.paintExtent - (parentMainAxisPosition - paintOffset.dx),
    };
  }

  @override
  int get indexOfFirstChild {
    assert(center != null);
    assert(center!.parent == this);
    assert(firstChild != null);
    int count = 0;
    RenderSliver? child = center;
    while (child != firstChild) {
      count -= 1;
      child = childBefore(child!);
    }
    return count;
  }

  @override
  String labelForChild(int index) {
    if (index == 0) {
      return 'center child';
    }
    return 'child $index';
  }

  @override
  Iterable<RenderSliver> get childrenInPaintOrder {
    final List<RenderSliver> children = <RenderSliver>[];
    if (firstChild == null) {
      return children;
    }
    RenderSliver? child = firstChild;
    while (child != center) {
      children.add(child!);
      child = childAfter(child);
    }
    child = lastChild;
    while (true) {
      children.add(child!);
      if (child == center) {
        return children;
      }
      child = childBefore(child);
    }
  }

  @override
  Iterable<RenderSliver> get childrenInHitTestOrder {
    final List<RenderSliver> children = <RenderSliver>[];
    if (firstChild == null) {
      return children;
    }
    RenderSliver? child = center;
    while (child != null) {
      children.add(child);
      child = childAfter(child);
    }
    child = childBefore(center!);
    while (child != null) {
      children.add(child);
      child = childBefore(child);
    }
    return children;
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DoubleProperty('anchor', anchor));
  }
}

/// A render object that is bigger on the inside and shrink wraps its children
/// in the main axis.
///
/// [RenderShrinkWrappingViewport] displays a subset of its children according
/// to its own dimensions and the given [offset]. As the offset varies, different
/// children are visible through the viewport.
///
/// [RenderShrinkWrappingViewport] differs from [RenderViewport] in that
/// [RenderViewport] expands to fill the main axis whereas
/// [RenderShrinkWrappingViewport] sizes itself to match its children in the
/// main axis. This shrink wrapping behavior is expensive because the children,
/// and hence the viewport, could potentially change size whenever the [offset]
/// changes (e.g., because of a collapsing header).
///
/// [RenderShrinkWrappingViewport] cannot contain [RenderBox] children directly.
/// Instead, use a [RenderSliverList], [RenderSliverFixedExtentList],
/// [RenderSliverGrid], or a [RenderSliverToBoxAdapter], for example.
///
/// See also:
///
///  * [RenderViewport], a viewport that does not shrink-wrap its contents.
///  * [RenderSliver], which explains more about the Sliver protocol.
///  * [RenderBox], which explains more about the Box protocol.
///  * [RenderSliverToBoxAdapter], which allows a [RenderBox] object to be
///    placed inside a [RenderSliver] (the opposite of this class).
class RenderShrinkWrappingViewport extends RenderViewportBase<SliverLogicalContainerParentData> {
  /// Creates a viewport (for [RenderSliver] objects) that shrink-wraps its
  /// contents.
  ///
  /// The [offset] must be specified. For testing purposes, consider passing a
  /// [ViewportOffset.zero] or [ViewportOffset.fixed].
  RenderShrinkWrappingViewport({
    super.axisDirection,
    required super.crossAxisDirection,
    required super.offset,
    super.clipBehavior,
    List<RenderSliver>? children,
  }) {
    addAll(children);
  }

  @override
  void setupParentData(RenderObject child) {
    if (child.parentData is! SliverLogicalContainerParentData) {
      child.parentData = SliverLogicalContainerParentData();
    }
  }

  @override
  bool debugThrowIfNotCheckingIntrinsics() {
    assert(() {
      if (!RenderObject.debugCheckingIntrinsics) {
        throw FlutterError.fromParts(<DiagnosticsNode>[
          ErrorSummary('$runtimeType does not support returning intrinsic dimensions.'),
          ErrorDescription(
            'Calculating the intrinsic dimensions would require instantiating every child of '
            'the viewport, which defeats the point of viewports being lazy.',
          ),
          ErrorHint(
            'If you are merely trying to shrink-wrap the viewport in the main axis direction, '
            'you should be able to achieve that effect by just giving the viewport loose '
            'constraints, without needing to measure its intrinsic dimensions.',
          ),
        ]);
      }
      return true;
    }());
    return true;
  }

  // Out-of-band data computed during layout.
  late double _maxScrollExtent;
  late double _shrinkWrapExtent;
  bool _hasVisualOverflow = false;

  bool _debugCheckHasBoundedCrossAxis() {
    assert(() {
      switch (axis) {
        case Axis.vertical:
          if (!constraints.hasBoundedWidth) {
            throw FlutterError(
              'Vertical viewport was given unbounded width.\n'
              'Viewports expand in the cross axis to fill their container and '
              'constrain their children to match their extent in the cross axis. '
              'In this case, a vertical shrinkwrapping viewport was given an '
              'unlimited amount of horizontal space in which to expand.',
            );
          }
        case Axis.horizontal:
          if (!constraints.hasBoundedHeight) {
            throw FlutterError(
              'Horizontal viewport was given unbounded height.\n'
              'Viewports expand in the cross axis to fill their container and '
              'constrain their children to match their extent in the cross axis. '
              'In this case, a horizontal shrinkwrapping viewport was given an '
              'unlimited amount of vertical space in which to expand.',
            );
          }
      }
      return true;
    }());
    return true;
  }

  @override
  void performLayout() {
    final BoxConstraints constraints = this.constraints;
    if (firstChild == null) {
      // Shrinkwrapping viewport only requires the cross axis to be bounded.
      assert(_debugCheckHasBoundedCrossAxis());
      size = switch (axis) {
        Axis.vertical => Size(constraints.maxWidth, constraints.minHeight),
        Axis.horizontal => Size(constraints.minWidth, constraints.maxHeight),
      };
      offset.applyViewportDimension(0.0);
      _maxScrollExtent = 0.0;
      _shrinkWrapExtent = 0.0;
      _hasVisualOverflow = false;
      offset.applyContentDimensions(0.0, 0.0);
      return;
    }

    // Shrinkwrapping viewport only requires the cross axis to be bounded.
    assert(_debugCheckHasBoundedCrossAxis());
    final (double mainAxisExtent, double crossAxisExtent) = switch (axis) {
      Axis.vertical => (constraints.maxHeight, constraints.maxWidth),
      Axis.horizontal => (constraints.maxWidth, constraints.maxHeight),
    };

    double correction;
    double effectiveExtent;
    while (true) {
      correction = _attemptLayout(mainAxisExtent, crossAxisExtent, offset.pixels);
      if (correction != 0.0) {
        offset.correctBy(correction);
      } else {
        effectiveExtent = switch (axis) {
          Axis.vertical => constraints.constrainHeight(_shrinkWrapExtent),
          Axis.horizontal => constraints.constrainWidth(_shrinkWrapExtent),
        };
        final bool didAcceptViewportDimension = offset.applyViewportDimension(effectiveExtent);
        final bool didAcceptContentDimension = offset.applyContentDimensions(
          0.0,
          math.max(0.0, _maxScrollExtent - effectiveExtent),
        );
        if (didAcceptViewportDimension && didAcceptContentDimension) {
          break;
        }
      }
    }
    size = switch (axis) {
      Axis.vertical => constraints.constrainDimensions(crossAxisExtent, effectiveExtent),
      Axis.horizontal => constraints.constrainDimensions(effectiveExtent, crossAxisExtent),
    };
  }

  double _attemptLayout(double mainAxisExtent, double crossAxisExtent, double correctedOffset) {
    // We can't assert mainAxisExtent is finite, because it could be infinite if
    // it is within a column or row for example. In such a case, there's not
    // even any scrolling to do, although some scroll physics (i.e.
    // BouncingScrollPhysics) could still temporarily scroll the content in a
    // simulation.
    assert(!mainAxisExtent.isNaN);
    assert(mainAxisExtent >= 0.0);
    assert(crossAxisExtent.isFinite);
    assert(crossAxisExtent >= 0.0);
    assert(correctedOffset.isFinite);
    _maxScrollExtent = 0.0;
    _shrinkWrapExtent = 0.0;
    // Since the viewport is shrinkwrapped, we know that any negative overscroll
    // into the potentially infinite mainAxisExtent will overflow the end of
    // the viewport.
    _hasVisualOverflow = correctedOffset < 0.0;
    _calculatedCacheExtent = switch (cacheExtentStyle) {
      CacheExtentStyle.pixel => cacheExtent,
      CacheExtentStyle.viewport => mainAxisExtent * _cacheExtent,
    };

    return layoutChildSequence(
      child: firstChild,
      scrollOffset: math.max(0.0, correctedOffset),
      overlap: math.min(0.0, correctedOffset),
      layoutOffset: math.max(0.0, -correctedOffset),
      remainingPaintExtent: mainAxisExtent + math.min(0.0, correctedOffset),
      mainAxisExtent: mainAxisExtent,
      crossAxisExtent: crossAxisExtent,
      growthDirection: GrowthDirection.forward,
      advance: childAfter,
      remainingCacheExtent: mainAxisExtent + 2 * _calculatedCacheExtent!,
      cacheOrigin: -_calculatedCacheExtent!,
    );
  }

  @override
  bool get hasVisualOverflow => _hasVisualOverflow;

  @override
  void updateOutOfBandData(GrowthDirection growthDirection, SliverGeometry childLayoutGeometry) {
    assert(growthDirection == GrowthDirection.forward);
    _maxScrollExtent += childLayoutGeometry.scrollExtent;
    if (childLayoutGeometry.hasVisualOverflow) {
      _hasVisualOverflow = true;
    }
    _shrinkWrapExtent += childLayoutGeometry.maxPaintExtent;
  }

  @override
  void updateChildLayoutOffset(
    RenderSliver child,
    double layoutOffset,
    GrowthDirection growthDirection,
  ) {
    assert(growthDirection == GrowthDirection.forward);
    final SliverLogicalParentData childParentData = child.parentData! as SliverLogicalParentData;
    childParentData.layoutOffset = layoutOffset;
  }

  @override
  Offset paintOffsetOf(RenderSliver child) {
    final SliverLogicalParentData childParentData = child.parentData! as SliverLogicalParentData;
    return computeAbsolutePaintOffset(
      child,
      childParentData.layoutOffset!,
      GrowthDirection.forward,
    );
  }

  @override
  double scrollOffsetOf(RenderSliver child, double scrollOffsetWithinChild) {
    assert(child.parent == this);
    assert(child.constraints.growthDirection == GrowthDirection.forward);
    double scrollOffsetToChild = 0.0;
    RenderSliver? current = firstChild;
    while (current != child) {
      scrollOffsetToChild += current!.geometry!.scrollExtent;
      current = childAfter(current);
    }
    return scrollOffsetToChild + scrollOffsetWithinChild;
  }

  @override
  double maxScrollObstructionExtentBefore(RenderSliver child) {
    assert(child.parent == this);
    assert(child.constraints.growthDirection == GrowthDirection.forward);
    double pinnedExtent = 0.0;
    RenderSliver? current = firstChild;
    while (current != child) {
      pinnedExtent += current!.geometry!.maxScrollObstructionExtent;
      current = childAfter(current);
    }
    return pinnedExtent;
  }

  @override
  void applyPaintTransform(RenderObject child, Matrix4 transform) {
    // Hit test logic relies on this always providing an invertible matrix.
    final Offset offset = paintOffsetOf(child as RenderSliver);
    transform.translate(offset.dx, offset.dy);
  }

  @override
  double computeChildMainAxisPosition(RenderSliver child, double parentMainAxisPosition) {
    assert(hasSize);
    final double layoutOffset = (child.parentData! as SliverLogicalParentData).layoutOffset!;
    return switch (applyGrowthDirectionToAxisDirection(
      child.constraints.axisDirection,
      child.constraints.growthDirection,
    )) {
      AxisDirection.down || AxisDirection.right => parentMainAxisPosition - layoutOffset,
      AxisDirection.up => size.height - parentMainAxisPosition - layoutOffset,
      AxisDirection.left => size.width - parentMainAxisPosition - layoutOffset,
    };
  }

  @override
  int get indexOfFirstChild => 0;

  @override
  String labelForChild(int index) => 'child $index';

  @override
  Iterable<RenderSliver> get childrenInPaintOrder {
    final List<RenderSliver> children = <RenderSliver>[];
    RenderSliver? child = lastChild;
    while (child != null) {
      children.add(child);
      child = childBefore(child);
    }
    return children;
  }

  @override
  Iterable<RenderSliver> get childrenInHitTestOrder {
    final List<RenderSliver> children = <RenderSliver>[];
    RenderSliver? child = firstChild;
    while (child != null) {
      children.add(child);
      child = childAfter(child);
    }
    return children;
  }
}
