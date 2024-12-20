// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'layout_builder.dart';
/// @docImport 'page_storage.dart';
/// @docImport 'page_view.dart';
library;

import 'dart:math' as math;

import 'package:flutter/gestures.dart' show DragStartBehavior;
import 'package:flutter/rendering.dart';

import 'basic.dart';
import 'focus_manager.dart';
import 'focus_scope.dart';
import 'framework.dart';
import 'notification_listener.dart';
import 'primary_scroll_controller.dart';
import 'scroll_controller.dart';
import 'scroll_notification.dart';
import 'scroll_physics.dart';
import 'scroll_view.dart';
import 'scrollable.dart';

/// A box in which a single widget can be scrolled.
///
/// This widget is useful when you have a single box that will normally be
/// entirely visible, for example a clock face in a time picker, but you need to
/// make sure it can be scrolled if the container gets too small in one axis
/// (the scroll direction).
///
/// It is also useful if you need to shrink-wrap in both axes (the main
/// scrolling direction as well as the cross axis), as one might see in a dialog
/// or pop-up menu. In that case, you might pair the [SingleChildScrollView]
/// with a [ListBody] child.
///
/// When you have a list of children and do not require cross-axis
/// shrink-wrapping behavior, for example a scrolling list that is always the
/// width of the screen, consider [ListView], which is vastly more efficient
/// than a [SingleChildScrollView] containing a [ListBody] or [Column] with
/// many children.
///
/// ## Sample code: Using [SingleChildScrollView] with a [Column]
///
/// Sometimes a layout is designed around the flexible properties of a
/// [Column], but there is the concern that in some cases, there might not
/// be enough room to see the entire contents. This could be because some
/// devices have unusually small screens, or because the application can
/// be used in landscape mode where the aspect ratio isn't what was
/// originally envisioned, or because the application is being shown in a
/// small window in split-screen mode. In any case, as a result, it might
/// make sense to wrap the layout in a [SingleChildScrollView].
///
/// Doing so, however, usually results in a conflict between the [Column],
/// which typically tries to grow as big as it can, and the [SingleChildScrollView],
/// which provides its children with an infinite amount of space.
///
/// To resolve this apparent conflict, there are a couple of techniques, as
/// discussed below. These techniques should only be used when the content is
/// normally expected to fit on the screen, so that the lazy instantiation of a
/// sliver-based [ListView] or [CustomScrollView] is not expected to provide any
/// performance benefit. If the viewport is expected to usually contain content
/// beyond the dimensions of the screen, then [SingleChildScrollView] would be
/// very expensive (in which case [ListView] may be a better choice than
/// [Column]).
///
/// ### Centering, spacing, or aligning fixed-height content
///
/// If the content has fixed (or intrinsic) dimensions but needs to be spaced out,
/// centered, or otherwise positioned using the [Flex] layout model of a [Column],
/// the following technique can be used to provide the [Column] with a minimum
/// dimension while allowing it to shrink-wrap the contents when there isn't enough
/// room to apply these spacing or alignment needs.
///
/// A [LayoutBuilder] is used to obtain the size of the viewport (implicitly via
/// the constraints that the [SingleChildScrollView] sees, since viewports
/// typically grow to fit their maximum height constraint). Then, inside the
/// scroll view, a [ConstrainedBox] is used to set the minimum height of the
/// [Column].
///
/// The [Column] has no [Expanded] children, so rather than take on the infinite
/// height from its [BoxConstraints.maxHeight], (the viewport provides no maximum height
/// constraint), it automatically tries to shrink to fit its children. It cannot
/// be smaller than its [BoxConstraints.minHeight], though, and It therefore
/// becomes the bigger of the minimum height provided by the
/// [ConstrainedBox] and the sum of the heights of the children.
///
/// If the children aren't enough to fit that minimum size, the [Column] ends up
/// with some remaining space to allocate as specified by its
/// [Column.mainAxisAlignment] argument.
///
/// {@tool dartpad}
/// In this example, the children are spaced out equally, unless there's no more
/// room, in which case they stack vertically and scroll.
///
/// When using this technique, [Expanded] and [Flexible] are not useful, because
/// in both cases the "available space" is infinite (since this is in a viewport).
/// The next section describes a technique for providing a maximum height constraint.
///
/// ** See code in examples/api/lib/widgets/single_child_scroll_view/single_child_scroll_view.0.dart **
/// {@end-tool}
///
/// ### Expanding content to fit the viewport
///
/// The following example builds on the previous one. In addition to providing a
/// minimum dimension for the child [Column], an [IntrinsicHeight] widget is used
/// to force the column to be exactly as big as its contents. This constraint
/// combines with the [ConstrainedBox] constraints discussed previously to ensure
/// that the column becomes either as big as viewport, or as big as the contents,
/// whichever is biggest.
///
/// Both constraints must be used to get the desired effect. If only the
/// [IntrinsicHeight] was specified, then the column would not grow to fit the
/// entire viewport when its children were smaller than the whole screen. If only
/// the size of the viewport was used, then the [Column] would overflow if the
/// children were bigger than the viewport.
///
/// The widget that is to grow to fit the remaining space so provided is wrapped
/// in an [Expanded] widget.
///
/// This technique is quite expensive, as it more or less requires that the contents
/// of the viewport be laid out twice (once to find their intrinsic dimensions, and
/// once to actually lay them out). The number of widgets within the column should
/// therefore be kept small. Alternatively, subsets of the children that have known
/// dimensions can be wrapped in a [SizedBox] that has tight vertical constraints,
/// so that the intrinsic sizing algorithm can short-circuit the computation when it
/// reaches those parts of the subtree.
///
/// {@tool dartpad}
/// In this example, the column becomes either as big as viewport, or as big as
/// the contents, whichever is biggest.
///
/// ** See code in examples/api/lib/widgets/single_child_scroll_view/single_child_scroll_view.1.dart **
/// {@end-tool}
///
/// {@macro flutter.widgets.ScrollView.PageStorage}
///
/// See also:
///
///  * [ListView], which handles multiple children in a scrolling list.
///  * [GridView], which handles multiple children in a scrolling grid.
///  * [PageView], for a scrollable that works page by page.
///  * [Scrollable], which handles arbitrary scrolling effects.
class SingleChildScrollView extends StatelessWidget {
  /// Creates a box in which a single widget can be scrolled.
  const SingleChildScrollView({
    super.key,
    this.scrollDirection = Axis.vertical,
    this.reverse = false,
    this.padding,
    this.primary,
    this.physics,
    this.controller,
    this.child,
    this.dragStartBehavior = DragStartBehavior.start,
    this.clipBehavior = Clip.hardEdge,
    this.hitTestBehavior = HitTestBehavior.opaque,
    this.restorationId,
    this.keyboardDismissBehavior = ScrollViewKeyboardDismissBehavior.manual,
  }) : assert(
         !(controller != null && (primary ?? false)),
         'Primary ScrollViews obtain their ScrollController via inheritance '
         'from a PrimaryScrollController widget. You cannot both set primary to '
         'true and pass an explicit controller.',
       );

  /// {@macro flutter.widgets.scroll_view.scrollDirection}
  final Axis scrollDirection;

  /// Whether the scroll view scrolls in the reading direction.
  ///
  /// For example, if the reading direction is left-to-right and
  /// [scrollDirection] is [Axis.horizontal], then the scroll view scrolls from
  /// left to right when [reverse] is false and from right to left when
  /// [reverse] is true.
  ///
  /// Similarly, if [scrollDirection] is [Axis.vertical], then the scroll view
  /// scrolls from top to bottom when [reverse] is false and from bottom to top
  /// when [reverse] is true.
  ///
  /// Defaults to false.
  final bool reverse;

  /// The amount of space by which to inset the child.
  final EdgeInsetsGeometry? padding;

  /// An object that can be used to control the position to which this scroll
  /// view is scrolled.
  ///
  /// Must be null if [primary] is true.
  ///
  /// A [ScrollController] serves several purposes. It can be used to control
  /// the initial scroll position (see [ScrollController.initialScrollOffset]).
  /// It can be used to control whether the scroll view should automatically
  /// save and restore its scroll position in the [PageStorage] (see
  /// [ScrollController.keepScrollOffset]). It can be used to read the current
  /// scroll position (see [ScrollController.offset]), or change it (see
  /// [ScrollController.animateTo]).
  final ScrollController? controller;

  /// {@macro flutter.widgets.scroll_view.primary}
  final bool? primary;

  /// How the scroll view should respond to user input.
  ///
  /// For example, determines how the scroll view continues to animate after the
  /// user stops dragging the scroll view.
  ///
  /// Defaults to matching platform conventions.
  final ScrollPhysics? physics;

  /// The widget that scrolls.
  ///
  /// {@macro flutter.widgets.ProxyWidget.child}
  final Widget? child;

  /// {@macro flutter.widgets.scrollable.dragStartBehavior}
  final DragStartBehavior dragStartBehavior;

  /// {@macro flutter.material.Material.clipBehavior}
  ///
  /// Defaults to [Clip.hardEdge].
  final Clip clipBehavior;

  /// {@macro flutter.widgets.scrollable.hitTestBehavior}
  ///
  /// Defaults to [HitTestBehavior.opaque].
  final HitTestBehavior hitTestBehavior;

  /// {@macro flutter.widgets.scrollable.restorationId}
  final String? restorationId;

  /// {@macro flutter.widgets.scroll_view.keyboardDismissBehavior}
  final ScrollViewKeyboardDismissBehavior keyboardDismissBehavior;

  AxisDirection _getDirection(BuildContext context) {
    return getAxisDirectionFromAxisReverseAndDirectionality(context, scrollDirection, reverse);
  }

  @override
  Widget build(BuildContext context) {
    final AxisDirection axisDirection = _getDirection(context);
    Widget? contents = child;
    if (padding != null) {
      contents = Padding(padding: padding!, child: contents);
    }
    final bool effectivePrimary =
        primary ??
        controller == null && PrimaryScrollController.shouldInherit(context, scrollDirection);

    final ScrollController? scrollController =
        effectivePrimary ? PrimaryScrollController.maybeOf(context) : controller;

    Widget scrollable = Scrollable(
      dragStartBehavior: dragStartBehavior,
      axisDirection: axisDirection,
      controller: scrollController,
      physics: physics,
      restorationId: restorationId,
      clipBehavior: clipBehavior,
      hitTestBehavior: hitTestBehavior,
      viewportBuilder: (BuildContext context, ViewportOffset offset) {
        return _SingleChildViewport(
          axisDirection: axisDirection,
          offset: offset,
          clipBehavior: clipBehavior,
          child: contents,
        );
      },
    );

    if (keyboardDismissBehavior == ScrollViewKeyboardDismissBehavior.onDrag) {
      scrollable = NotificationListener<ScrollUpdateNotification>(
        child: scrollable,
        onNotification: (ScrollUpdateNotification notification) {
          final FocusScopeNode currentScope = FocusScope.of(context);
          if (!currentScope.hasPrimaryFocus && currentScope.hasFocus) {
            FocusManager.instance.primaryFocus?.unfocus();
          }
          return false;
        },
      );
    }

    return effectivePrimary && scrollController != null
        // Further descendant ScrollViews will not inherit the same
        // PrimaryScrollController
        ? PrimaryScrollController.none(child: scrollable)
        : scrollable;
  }
}

class _SingleChildViewport extends SingleChildRenderObjectWidget {
  const _SingleChildViewport({
    this.axisDirection = AxisDirection.down,
    required this.offset,
    super.child,
    required this.clipBehavior,
  });

  final AxisDirection axisDirection;
  final ViewportOffset offset;
  final Clip clipBehavior;

  @override
  _RenderSingleChildViewport createRenderObject(BuildContext context) {
    return _RenderSingleChildViewport(
      axisDirection: axisDirection,
      offset: offset,
      clipBehavior: clipBehavior,
    );
  }

  @override
  void updateRenderObject(BuildContext context, _RenderSingleChildViewport renderObject) {
    // Order dependency: The offset setter reads the axis direction.
    renderObject
      ..axisDirection = axisDirection
      ..offset = offset
      ..clipBehavior = clipBehavior;
  }

  @override
  SingleChildRenderObjectElement createElement() {
    return _SingleChildViewportElement(this);
  }
}

class _SingleChildViewportElement extends SingleChildRenderObjectElement
    with NotifiableElementMixin, ViewportElementMixin {
  _SingleChildViewportElement(_SingleChildViewport super.widget);
}

class _RenderSingleChildViewport extends RenderBox
    with RenderObjectWithChildMixin<RenderBox>
    implements RenderAbstractViewport {
  _RenderSingleChildViewport({
    AxisDirection axisDirection = AxisDirection.down,
    required ViewportOffset offset,
    RenderBox? child,
    required Clip clipBehavior,
  }) : _axisDirection = axisDirection,
       _offset = offset,
       _clipBehavior = clipBehavior {
    this.child = child;
  }

  AxisDirection get axisDirection => _axisDirection;
  AxisDirection _axisDirection;
  set axisDirection(AxisDirection value) {
    if (value == _axisDirection) {
      return;
    }
    _axisDirection = value;
    markNeedsLayout();
  }

  Axis get axis => axisDirectionToAxis(axisDirection);

  ViewportOffset get offset => _offset;
  ViewportOffset _offset;
  set offset(ViewportOffset value) {
    if (value == _offset) {
      return;
    }
    if (attached) {
      _offset.removeListener(_hasScrolled);
    }
    _offset = value;
    if (attached) {
      _offset.addListener(_hasScrolled);
    }
    markNeedsLayout();
  }

  /// {@macro flutter.material.Material.clipBehavior}
  ///
  /// Defaults to [Clip.none].
  Clip get clipBehavior => _clipBehavior;
  Clip _clipBehavior = Clip.none;
  set clipBehavior(Clip value) {
    if (value != _clipBehavior) {
      _clipBehavior = value;
      markNeedsPaint();
      markNeedsSemanticsUpdate();
    }
  }

  void _hasScrolled() {
    markNeedsPaint();
    markNeedsSemanticsUpdate();
  }

  @override
  void setupParentData(RenderObject child) {
    // We don't actually use the offset argument in BoxParentData, so let's
    // avoid allocating it at all.
    if (child.parentData is! ParentData) {
      child.parentData = ParentData();
    }
  }

  @override
  void attach(PipelineOwner owner) {
    super.attach(owner);
    _offset.addListener(_hasScrolled);
  }

  @override
  void detach() {
    _offset.removeListener(_hasScrolled);
    super.detach();
  }

  @override
  bool get isRepaintBoundary => true;

  double get _viewportExtent {
    assert(hasSize);
    return switch (axis) {
      Axis.horizontal => size.width,
      Axis.vertical => size.height,
    };
  }

  double get _minScrollExtent {
    assert(hasSize);
    return 0.0;
  }

  double get _maxScrollExtent {
    assert(hasSize);
    if (child == null) {
      return 0.0;
    }
    return math.max(0.0, switch (axis) {
      Axis.horizontal => child!.size.width - size.width,
      Axis.vertical => child!.size.height - size.height,
    });
  }

  BoxConstraints _getInnerConstraints(BoxConstraints constraints) {
    return switch (axis) {
      Axis.horizontal => constraints.heightConstraints(),
      Axis.vertical => constraints.widthConstraints(),
    };
  }

  @override
  double computeMinIntrinsicWidth(double height) {
    return child?.getMinIntrinsicWidth(height) ?? 0.0;
  }

  @override
  double computeMaxIntrinsicWidth(double height) {
    return child?.getMaxIntrinsicWidth(height) ?? 0.0;
  }

  @override
  double computeMinIntrinsicHeight(double width) {
    return child?.getMinIntrinsicHeight(width) ?? 0.0;
  }

  @override
  double computeMaxIntrinsicHeight(double width) {
    return child?.getMaxIntrinsicHeight(width) ?? 0.0;
  }

  // We don't override computeDistanceToActualBaseline(), because we
  // want the default behavior (returning null). Otherwise, as you
  // scroll, it would shift in its parent if the parent was baseline-aligned,
  // which makes no sense.

  @override
  Size computeDryLayout(BoxConstraints constraints) {
    if (child == null) {
      return constraints.smallest;
    }
    final Size childSize = child!.getDryLayout(_getInnerConstraints(constraints));
    return constraints.constrain(childSize);
  }

  @override
  void performLayout() {
    final BoxConstraints constraints = this.constraints;
    if (child == null) {
      size = constraints.smallest;
    } else {
      child!.layout(_getInnerConstraints(constraints), parentUsesSize: true);
      size = constraints.constrain(child!.size);
    }

    if (offset.hasPixels) {
      if (offset.pixels > _maxScrollExtent) {
        offset.correctBy(_maxScrollExtent - offset.pixels);
      } else if (offset.pixels < _minScrollExtent) {
        offset.correctBy(_minScrollExtent - offset.pixels);
      }
    }

    offset.applyViewportDimension(_viewportExtent);
    offset.applyContentDimensions(_minScrollExtent, _maxScrollExtent);
  }

  Offset get _paintOffset => _paintOffsetForPosition(offset.pixels);

  Offset _paintOffsetForPosition(double position) {
    return switch (axisDirection) {
      AxisDirection.up => Offset(0.0, position - child!.size.height + size.height),
      AxisDirection.left => Offset(position - child!.size.width + size.width, 0.0),
      AxisDirection.right => Offset(-position, 0.0),
      AxisDirection.down => Offset(0.0, -position),
    };
  }

  bool _shouldClipAtPaintOffset(Offset paintOffset) {
    assert(child != null);
    switch (clipBehavior) {
      case Clip.none:
        return false;
      case Clip.hardEdge:
      case Clip.antiAlias:
      case Clip.antiAliasWithSaveLayer:
        return paintOffset.dx < 0 ||
            paintOffset.dy < 0 ||
            paintOffset.dx + child!.size.width > size.width ||
            paintOffset.dy + child!.size.height > size.height;
    }
  }

  @override
  void paint(PaintingContext context, Offset offset) {
    if (child != null) {
      final Offset paintOffset = _paintOffset;

      void paintContents(PaintingContext context, Offset offset) {
        context.paintChild(child!, offset + paintOffset);
      }

      if (_shouldClipAtPaintOffset(paintOffset)) {
        _clipRectLayer.layer = context.pushClipRect(
          needsCompositing,
          offset,
          Offset.zero & size,
          paintContents,
          clipBehavior: clipBehavior,
          oldLayer: _clipRectLayer.layer,
        );
      } else {
        _clipRectLayer.layer = null;
        paintContents(context, offset);
      }
    }
  }

  final LayerHandle<ClipRectLayer> _clipRectLayer = LayerHandle<ClipRectLayer>();

  @override
  void dispose() {
    _clipRectLayer.layer = null;
    super.dispose();
  }

  @override
  void applyPaintTransform(RenderBox child, Matrix4 transform) {
    final Offset paintOffset = _paintOffset;
    transform.translate(paintOffset.dx, paintOffset.dy);
  }

  @override
  Rect? describeApproximatePaintClip(RenderObject? child) {
    if (child != null && _shouldClipAtPaintOffset(_paintOffset)) {
      return Offset.zero & size;
    }
    return null;
  }

  @override
  bool hitTestChildren(BoxHitTestResult result, {required Offset position}) {
    if (child != null) {
      return result.addWithPaintOffset(
        offset: _paintOffset,
        position: position,
        hitTest: (BoxHitTestResult result, Offset transformed) {
          assert(transformed == position + -_paintOffset);
          return child!.hitTest(result, position: transformed);
        },
      );
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

    rect ??= target.paintBounds;
    if (target is! RenderBox) {
      return RevealedOffset(offset: offset.pixels, rect: rect);
    }

    final RenderBox targetBox = target;
    final Matrix4 transform = targetBox.getTransformTo(child);
    final Rect bounds = MatrixUtils.transformRect(transform, rect);
    final Size contentSize = child!.size;

    final (
      double mainAxisExtent,
      double leadingScrollOffset,
      double targetMainAxisExtent,
    ) = switch (axisDirection) {
      AxisDirection.up => (size.height, contentSize.height - bounds.bottom, bounds.height),
      AxisDirection.left => (size.width, contentSize.width - bounds.right, bounds.width),
      AxisDirection.right => (size.width, bounds.left, bounds.width),
      AxisDirection.down => (size.height, bounds.top, bounds.height),
    };

    final double targetOffset =
        leadingScrollOffset - (mainAxisExtent - targetMainAxisExtent) * alignment;
    final Rect targetRect = bounds.shift(_paintOffsetForPosition(targetOffset));
    return RevealedOffset(offset: targetOffset, rect: targetRect);
  }

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

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DiagnosticsProperty<Offset>('offset', _paintOffset));
  }

  @override
  Rect describeSemanticsClip(RenderObject child) {
    final double remainingOffset = _maxScrollExtent - offset.pixels;
    switch (axisDirection) {
      case AxisDirection.up:
        return Rect.fromLTRB(
          semanticBounds.left,
          semanticBounds.top - remainingOffset,
          semanticBounds.right,
          semanticBounds.bottom + offset.pixels,
        );
      case AxisDirection.right:
        return Rect.fromLTRB(
          semanticBounds.left - offset.pixels,
          semanticBounds.top,
          semanticBounds.right + remainingOffset,
          semanticBounds.bottom,
        );
      case AxisDirection.down:
        return Rect.fromLTRB(
          semanticBounds.left,
          semanticBounds.top - offset.pixels,
          semanticBounds.right,
          semanticBounds.bottom + remainingOffset,
        );
      case AxisDirection.left:
        return Rect.fromLTRB(
          semanticBounds.left - remainingOffset,
          semanticBounds.top,
          semanticBounds.right + offset.pixels,
          semanticBounds.bottom,
        );
    }
  }
}
