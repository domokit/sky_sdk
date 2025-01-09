// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/material.dart';
library;

import 'dart:collection';
import 'dart:math' as math show pi;
import 'dart:ui' as ui;

import 'package:flutter/foundation.dart' show Brightness, clampDouble;
import 'package:flutter/rendering.dart';
import 'package:flutter/widgets.dart';

import 'colors.dart';
import 'text_selection_toolbar_button.dart';
import 'theme.dart';

// The radius of the toolbar RRect shape.
// Value extracted from https://developer.apple.com/design/resources/.
const Radius _kToolbarBorderRadius = Radius.circular(8.0);

// Vertical distance between the tip of the arrow and the line of text the arrow
// is pointing to. The value used here is eyeballed.
const double _kToolbarContentDistance = 8.0;

// The size of the arrow pointing to the anchor. Eyeballed value.
const Size _kToolbarArrowSize = Size(14.0, 7.0);

// Minimal padding from tip of the selection toolbar arrow to horizontal edges of the
// screen. Eyeballed value.
const double _kArrowScreenPadding = 26.0;

// The size and thickness of the chevron icon used for navigating between toolbar pages.
// Eyeballed values.
const double _kToolbarChevronSize = 10.0;
const double _kToolbarChevronThickness = 2.0;

// Color was measured from a screenshot of iOS 16.0.2
// TODO(LongCatIsLooong): https://github.com/flutter/flutter/issues/41507.
const CupertinoDynamicColor _kToolbarBackgroundColor = CupertinoDynamicColor.withBrightness(
  color: Color(0xFFF6F6F6),
  darkColor: Color(0xFF222222),
);

// Color was measured from a screenshot of iOS 16.0.2.
const CupertinoDynamicColor _kToolbarDividerColor = CupertinoDynamicColor.withBrightness(
  color: Color(0xFFD6D6D6),
  darkColor: Color(0xFF424242),
);

const CupertinoDynamicColor _kToolbarTextColor = CupertinoDynamicColor.withBrightness(
  color: CupertinoColors.black,
  darkColor: CupertinoColors.white,
);

const Duration _kToolbarTransitionDuration = Duration(milliseconds: 125);

/// The type for a Function that builds a toolbar's container with the given
/// child.
///
/// The anchor is provided in global coordinates.
///
/// See also:
///
///   * [CupertinoTextSelectionToolbar.toolbarBuilder], which is of this type.
///   * [TextSelectionToolbar.toolbarBuilder], which is similar, but for an
///     Material-style toolbar.
typedef CupertinoToolbarBuilder =
    Widget Function(BuildContext context, Offset anchorAbove, Offset anchorBelow, Widget child);

/// An iOS-style text selection toolbar.
///
/// Typically displays buttons for text manipulation, e.g. copying and pasting
/// text.
///
/// Tries to position itself above [anchorAbove], but if it doesn't fit, then
/// it positions itself below [anchorBelow].
///
/// If any children don't fit in the menu, an overflow menu will automatically
/// be created.
///
/// See also:
///
///  * [AdaptiveTextSelectionToolbar], which builds the toolbar for the current
///    platform.
///  * [TextSelectionToolbar], which is similar, but builds an Android-style
///    toolbar.
class CupertinoTextSelectionToolbar extends StatelessWidget {
  /// Creates an instance of CupertinoTextSelectionToolbar.
  const CupertinoTextSelectionToolbar({
    super.key,
    required this.anchorAbove,
    required this.anchorBelow,
    required this.children,
    this.toolbarBuilder = _defaultToolbarBuilder,
  }) : assert(children.length > 0);

  /// {@macro flutter.material.TextSelectionToolbar.anchorAbove}
  final Offset anchorAbove;

  /// {@macro flutter.material.TextSelectionToolbar.anchorBelow}
  final Offset anchorBelow;

  /// {@macro flutter.material.TextSelectionToolbar.children}
  ///
  /// See also:
  ///   * [CupertinoTextSelectionToolbarButton], which builds a default
  ///     Cupertino-style text selection toolbar text button.
  final List<Widget> children;

  /// {@macro flutter.material.TextSelectionToolbar.toolbarBuilder}
  ///
  /// The given anchor and isAbove can be used to position an arrow, as in the
  /// default Cupertino toolbar.
  final CupertinoToolbarBuilder toolbarBuilder;

  /// Minimal padding from all edges of the selection toolbar to all edges of the
  /// viewport.
  ///
  /// See also:
  ///
  ///  * [SpellCheckSuggestionsToolbar], which uses this same value for its
  ///    padding from the edges of the viewport.
  ///  * [TextSelectionToolbar], which uses this same value as well.
  static const double kToolbarScreenPadding = 8.0;

  // Builds a toolbar just like the default iOS toolbar, with the right color
  // background and a rounded cutout with an arrow.
  static Widget _defaultToolbarBuilder(
    BuildContext context,
    Offset anchorAbove,
    Offset anchorBelow,
    Widget child,
  ) {
    return _CupertinoTextSelectionToolbarShape(
      anchorAbove: anchorAbove,
      anchorBelow: anchorBelow,
      shadowColor:
          CupertinoTheme.brightnessOf(context) == Brightness.light
              ? CupertinoColors.black.withOpacity(0.2)
              : null,
      child: ColoredBox(color: _kToolbarBackgroundColor.resolveFrom(context), child: child),
    );
  }

  @override
  Widget build(BuildContext context) {
    assert(debugCheckHasMediaQuery(context));
    final EdgeInsets mediaQueryPadding = MediaQuery.paddingOf(context);

    final double paddingAbove = mediaQueryPadding.top + kToolbarScreenPadding;

    // The arrow, which points to the anchor, has some margin so it can't get
    // too close to the horizontal edges of the screen.
    final double leftMargin = _kArrowScreenPadding + mediaQueryPadding.left;
    final double rightMargin =
        MediaQuery.sizeOf(context).width - mediaQueryPadding.right - _kArrowScreenPadding;

    final Offset anchorAboveAdjusted = Offset(
      clampDouble(anchorAbove.dx, leftMargin, rightMargin),
      anchorAbove.dy - _kToolbarContentDistance - paddingAbove,
    );
    final Offset anchorBelowAdjusted = Offset(
      clampDouble(anchorBelow.dx, leftMargin, rightMargin),
      anchorBelow.dy + _kToolbarContentDistance - paddingAbove,
    );

    return Padding(
      padding: EdgeInsets.fromLTRB(
        kToolbarScreenPadding,
        paddingAbove,
        kToolbarScreenPadding,
        kToolbarScreenPadding,
      ),
      child: CustomSingleChildLayout(
        delegate: TextSelectionToolbarLayoutDelegate(
          anchorAbove: anchorAboveAdjusted,
          anchorBelow: anchorBelowAdjusted,
        ),
        child: _CupertinoTextSelectionToolbarContent(
          anchorAbove: anchorAboveAdjusted,
          anchorBelow: anchorBelowAdjusted,
          toolbarBuilder: toolbarBuilder,
          children: children,
        ),
      ),
    );
  }
}

// Clips the child so that it has the shape of the default iOS text selection
// toolbar, with rounded corners and an arrow pointing at the anchor.
//
// The anchor should be in global coordinates.
class _CupertinoTextSelectionToolbarShape extends SingleChildRenderObjectWidget {
  const _CupertinoTextSelectionToolbarShape({
    required Offset anchorAbove,
    required Offset anchorBelow,
    Color? shadowColor,
    super.child,
  }) : _anchorAbove = anchorAbove,
       _anchorBelow = anchorBelow,
       _shadowColor = shadowColor;

  final Offset _anchorAbove;
  final Offset _anchorBelow;
  final Color? _shadowColor;

  @override
  _RenderCupertinoTextSelectionToolbarShape createRenderObject(BuildContext context) =>
      _RenderCupertinoTextSelectionToolbarShape(_anchorAbove, _anchorBelow, _shadowColor, null);

  @override
  void updateRenderObject(
    BuildContext context,
    _RenderCupertinoTextSelectionToolbarShape renderObject,
  ) {
    renderObject
      ..anchorAbove = _anchorAbove
      ..anchorBelow = _anchorBelow
      ..shadowColor = _shadowColor;
  }
}

// Clips the child into the shape of the default iOS text selection toolbar.
//
// The shape is a rounded rectangle with a protruding arrow pointing at the
// given anchor in the direction indicated by isAbove.
//
// In order to allow the child to render itself independent of isAbove, its
// height is clipped on both the top and the bottom, leaving the arrow remaining
// on the necessary side.
class _RenderCupertinoTextSelectionToolbarShape extends RenderShiftedBox {
  _RenderCupertinoTextSelectionToolbarShape(
    this._anchorAbove,
    this._anchorBelow,
    this._shadowColor,
    super.child,
  );

  @override
  bool get isRepaintBoundary => true;

  Offset get anchorAbove => _anchorAbove;
  Offset _anchorAbove;
  set anchorAbove(Offset value) {
    if (value == _anchorAbove) {
      return;
    }
    _anchorAbove = value;
    markNeedsLayout();
  }

  Offset get anchorBelow => _anchorBelow;
  Offset _anchorBelow;
  set anchorBelow(Offset value) {
    if (value == _anchorBelow) {
      return;
    }
    _anchorBelow = value;
    markNeedsLayout();
  }

  Color? get shadowColor => _shadowColor;
  Color? _shadowColor;
  set shadowColor(Color? value) {
    if (value == _shadowColor) {
      return;
    }
    _shadowColor = value;
    markNeedsPaint();
  }

  bool _isAbove(double childHeight) =>
      anchorAbove.dy >= childHeight - _kToolbarArrowSize.height * 2;

  BoxConstraints _constraintsForChild(BoxConstraints constraints) {
    return BoxConstraints(
      minWidth: _kToolbarArrowSize.width + _kToolbarBorderRadius.x * 2,
    ).enforce(constraints.loosen());
  }

  Offset _computeChildOffset(Size childSize) {
    return Offset(0.0, _isAbove(childSize.height) ? -_kToolbarArrowSize.height : 0.0);
  }

  @override
  double? computeDryBaseline(covariant BoxConstraints constraints, TextBaseline baseline) {
    final RenderBox? child = this.child;
    if (child == null) {
      return null;
    }
    final BoxConstraints enforcedConstraint = _constraintsForChild(constraints);
    final double? result = child.getDryBaseline(enforcedConstraint, baseline);
    return result == null
        ? null
        : result + _computeChildOffset(child.getDryLayout(enforcedConstraint)).dy;
  }

  @override
  void performLayout() {
    final RenderBox? child = this.child;
    if (child == null) {
      return;
    }

    child.layout(_constraintsForChild(constraints), parentUsesSize: true);
    // The buttons are padded on both top and bottom sufficiently to have
    // the arrow clipped out of it on either side. By
    // using this approach, the buttons don't need any special padding that
    // depends on isAbove.
    // The height of one arrow will be clipped off of the child, so adjust the
    // size and position to remove that piece from the layout.
    final BoxParentData childParentData = child.parentData! as BoxParentData;
    childParentData.offset = _computeChildOffset(child.size);
    size = Size(child.size.width, child.size.height - _kToolbarArrowSize.height);
  }

  // Returns the RRect inside which the child is painted.
  RRect _shapeRRect(RenderBox child) {
    final Rect rect =
        Offset(0.0, _kToolbarArrowSize.height) &
        Size(child.size.width, child.size.height - _kToolbarArrowSize.height * 2);
    return RRect.fromRectAndRadius(rect, _kToolbarBorderRadius).scaleRadii();
  }

  // Adds the given `rrect` to the current `path`, starting from the last point
  // in `path` and ends after the last corner of the rrect (closest corner to
  // `startAngle` in the counterclockwise direction), without closing the path.
  //
  // The `startAngle` argument must be a multiple of pi / 2, with 0 being the
  // positive half of the x-axis, and pi / 2 being the negative half of the
  // y-axis.
  //
  // For instance, if `startAngle` equals pi/2 then this method draws a line
  // segment to the bottom-left corner of `rrect` from the last point in `path`,
  // and follows the `rrect` path clockwise until the bottom-right corner is
  // added, then this method returns the mutated path without closing it.
  static Path _addRRectToPath(Path path, RRect rrect, {required double startAngle}) {
    const double halfPI = math.pi / 2;
    assert(startAngle % halfPI == 0.0);
    final Rect rect = rrect.outerRect;

    final List<(Offset, Radius)> rrectCorners = <(Offset, Radius)>[
      (rect.bottomRight, -rrect.brRadius),
      (rect.bottomLeft, Radius.elliptical(rrect.blRadiusX, -rrect.blRadiusY)),
      (rect.topLeft, rrect.tlRadius),
      (rect.topRight, Radius.elliptical(-rrect.trRadiusX, rrect.trRadiusY)),
    ];

    // Add the 4 corners to the path clockwise. Convert radians to quadrants
    // to avoid fp arithmetics. The order is br -> bl -> tl -> tr if the starting
    // angle is 0.
    final int startQuadrantIndex = startAngle ~/ halfPI;
    for (int i = startQuadrantIndex; i < rrectCorners.length + startQuadrantIndex; i += 1) {
      final (Offset vertex, Radius rectCenterOffset) = rrectCorners[i % rrectCorners.length];
      final Offset otherVertex = Offset(
        vertex.dx + 2 * rectCenterOffset.x,
        vertex.dy + 2 * rectCenterOffset.y,
      );
      final Rect rect = Rect.fromPoints(vertex, otherVertex);
      path.arcTo(rect, halfPI * i, halfPI, false);
    }
    return path;
  }

  // The path is described in the toolbar child's coordinate system.
  Path _clipPath(RenderBox child, RRect rrect) {
    final Path path = Path();
    // If there isn't enough width for the arrow + radii, ignore the arrow.
    // Because of the constraints we gave children in performLayout, this should
    // only happen if the parent isn't wide enough which should be very rare, and
    // when that happens the arrow won't be too useful anyways.
    if (_kToolbarBorderRadius.x * 2 + _kToolbarArrowSize.width > size.width) {
      return path..addRRect(rrect);
    }

    final bool isAbove = _isAbove(child.size.height);
    final Offset localAnchor = globalToLocal(isAbove ? _anchorAbove : _anchorBelow);
    final double arrowTipX = clampDouble(
      localAnchor.dx,
      _kToolbarBorderRadius.x + _kToolbarArrowSize.width / 2,
      size.width - _kToolbarArrowSize.width / 2 - _kToolbarBorderRadius.x,
    );

    // Draw the path clockwise, starting from the beginning side of the arrow.
    if (isAbove) {
      final double arrowBaseY = child.size.height - _kToolbarArrowSize.height;
      final double arrowTipY = child.size.height;
      path
        ..moveTo(
          arrowTipX + _kToolbarArrowSize.width / 2,
          arrowBaseY,
        ) // right side of the arrow triangle
        ..lineTo(arrowTipX, arrowTipY) // The tip of the arrow
        ..lineTo(
          arrowTipX - _kToolbarArrowSize.width / 2,
          arrowBaseY,
        ); // left side of the arrow triangle
    } else {
      final double arrowBaseY = _kToolbarArrowSize.height;
      const double arrowTipY = 0.0;
      path
        ..moveTo(
          arrowTipX - _kToolbarArrowSize.width / 2,
          arrowBaseY,
        ) // right side of the arrow triangle
        ..lineTo(arrowTipX, arrowTipY) // The tip of the arrow
        ..lineTo(
          arrowTipX + _kToolbarArrowSize.width / 2,
          arrowBaseY,
        ); // left side of the arrow triangle
    }
    final double startAngle = isAbove ? math.pi / 2 : -math.pi / 2;
    return _addRRectToPath(path, rrect, startAngle: startAngle)..close();
  }

  @override
  void paint(PaintingContext context, Offset offset) {
    final RenderBox? child = this.child;
    if (child == null) {
      return;
    }

    final BoxParentData childParentData = child.parentData! as BoxParentData;

    final RRect rrect = _shapeRRect(child);
    final Path clipPath = _clipPath(child, rrect);

    // If configured, paint the shadow beneath the shape.
    if (_shadowColor != null) {
      final BoxShadow boxShadow = BoxShadow(color: _shadowColor!, blurRadius: 15.0);
      final RRect shadowRRect = RRect.fromLTRBR(
        rrect.left,
        rrect.top,
        rrect.right,
        rrect.bottom + _kToolbarArrowSize.height,
        _kToolbarBorderRadius,
      ).shift(offset + childParentData.offset + boxShadow.offset);
      context.canvas.drawRRect(shadowRRect, boxShadow.toPaint());
    }

    _clipPathLayer.layer = context.pushClipPath(
      needsCompositing,
      offset + childParentData.offset,
      Offset.zero & child.size,
      clipPath,
      (PaintingContext innerContext, Offset innerOffset) =>
          innerContext.paintChild(child, innerOffset),
      oldLayer: _clipPathLayer.layer,
    );
  }

  final LayerHandle<ClipPathLayer> _clipPathLayer = LayerHandle<ClipPathLayer>();
  Paint? _debugPaint;

  @override
  void dispose() {
    _clipPathLayer.layer = null;
    super.dispose();
  }

  @override
  void debugPaintSize(PaintingContext context, Offset offset) {
    assert(() {
      final RenderBox? child = this.child;
      if (child == null) {
        return true;
      }

      final ui.Paint debugPaint =
          _debugPaint ??=
              Paint()
                ..shader = ui.Gradient.linear(
                  Offset.zero,
                  const Offset(10.0, 10.0),
                  const <Color>[
                    CupertinoColors.transparent,
                    Color(0xFFFF00FF),
                    Color(0xFFFF00FF),
                    CupertinoColors.transparent,
                  ],
                  const <double>[0.25, 0.25, 0.75, 0.75],
                  TileMode.repeated,
                )
                ..strokeWidth = 2.0
                ..style = PaintingStyle.stroke;

      final BoxParentData childParentData = child.parentData! as BoxParentData;
      final Path clipPath = _clipPath(child, _shapeRRect(child));
      context.canvas.drawPath(clipPath.shift(offset + childParentData.offset), debugPaint);
      return true;
    }());
  }

  @override
  bool hitTestChildren(BoxHitTestResult result, {required Offset position}) {
    final RenderBox? child = this.child;
    if (child == null) {
      return false;
    }

    // Positions outside of the clipped area of the child are not counted as
    // hits.
    final BoxParentData childParentData = child.parentData! as BoxParentData;
    final Rect hitBox = Rect.fromLTWH(
      childParentData.offset.dx,
      childParentData.offset.dy + _kToolbarArrowSize.height,
      child.size.width,
      child.size.height - _kToolbarArrowSize.height * 2,
    );
    if (!hitBox.contains(position)) {
      return false;
    }

    return super.hitTestChildren(result, position: position);
  }
}

// A toolbar containing the given children. If they overflow the width
// available, then the menu will be paginated with the overflowing children
// displayed on subsequent pages.
//
// The anchor should be in global coordinates.
class _CupertinoTextSelectionToolbarContent extends StatefulWidget {
  const _CupertinoTextSelectionToolbarContent({
    required this.anchorAbove,
    required this.anchorBelow,
    required this.toolbarBuilder,
    required this.children,
  }) : assert(children.length > 0);

  final Offset anchorAbove;
  final Offset anchorBelow;
  final List<Widget> children;
  final CupertinoToolbarBuilder toolbarBuilder;

  @override
  _CupertinoTextSelectionToolbarContentState createState() =>
      _CupertinoTextSelectionToolbarContentState();
}

class _CupertinoTextSelectionToolbarContentState
    extends State<_CupertinoTextSelectionToolbarContent>
    with TickerProviderStateMixin {
  // Controls the fading of the buttons within the menu during page transitions.
  late AnimationController _controller;
  int? _nextPage;
  int _page = 0;

  final GlobalKey _toolbarItemsKey = GlobalKey();

  void _onHorizontalDragEnd(DragEndDetails details) {
    final double? velocity = details.primaryVelocity;

    if (velocity != null && velocity != 0) {
      if (velocity > 0) {
        _handlePreviousPage();
      } else {
        _handleNextPage();
      }
    }
  }

  void _handleNextPage() {
    final RenderBox? renderToolbar =
        _toolbarItemsKey.currentContext?.findRenderObject() as RenderBox?;

    if (renderToolbar is _RenderCupertinoTextSelectionToolbarItems && renderToolbar.hasNextPage) {
      _controller.reverse();
      _controller.addStatusListener(_statusListener);
      _nextPage = _page + 1;
    }
  }

  void _handlePreviousPage() {
    final RenderBox? renderToolbar =
        _toolbarItemsKey.currentContext?.findRenderObject() as RenderBox?;

    if (renderToolbar is _RenderCupertinoTextSelectionToolbarItems &&
        renderToolbar.hasPreviousPage) {
      _controller.reverse();
      _controller.addStatusListener(_statusListener);
      _nextPage = _page - 1;
    }
  }

  void _statusListener(AnimationStatus status) {
    if (!status.isDismissed) {
      return;
    }

    setState(() {
      _page = _nextPage!;
      _nextPage = null;
    });
    _controller.forward();
    _controller.removeStatusListener(_statusListener);
  }

  @override
  void initState() {
    super.initState();
    _controller = AnimationController(
      value: 1.0,
      vsync: this,
      // This was eyeballed on a physical iOS device running iOS 13.
      duration: _kToolbarTransitionDuration,
    );
  }

  @override
  void didUpdateWidget(_CupertinoTextSelectionToolbarContent oldWidget) {
    super.didUpdateWidget(oldWidget);
    // If the children are changing, the current page should be reset.
    if (widget.children != oldWidget.children) {
      _page = 0;
      _nextPage = null;
      _controller.forward();
      _controller.removeStatusListener(_statusListener);
    }
  }

  @override
  void dispose() {
    _controller.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final Color chevronColor = _kToolbarTextColor.resolveFrom(context);

    // Wrap the children and the chevron painters in Center with widthFactor
    // and heightFactor of 1.0 so _CupertinoTextSelectionToolbarItems can get
    // the natural size of the buttons and then expand vertically as needed.
    final Widget backButton = Center(
      widthFactor: 1.0,
      heightFactor: 1.0,
      child: CupertinoTextSelectionToolbarButton(
        onPressed: _handlePreviousPage,
        child: IgnorePointer(
          child: CustomPaint(
            painter: _LeftCupertinoChevronPainter(color: chevronColor),
            size: const Size.square(_kToolbarChevronSize),
          ),
        ),
      ),
    );
    final Widget nextButton = Center(
      widthFactor: 1.0,
      heightFactor: 1.0,
      child: CupertinoTextSelectionToolbarButton(
        onPressed: _handleNextPage,
        child: IgnorePointer(
          child: CustomPaint(
            painter: _RightCupertinoChevronPainter(color: chevronColor),
            size: const Size.square(_kToolbarChevronSize),
          ),
        ),
      ),
    );
    final List<Widget> children =
        widget.children.map((Widget child) {
          return Center(widthFactor: 1.0, heightFactor: 1.0, child: child);
        }).toList();

    return widget.toolbarBuilder(
      context,
      widget.anchorAbove,
      widget.anchorBelow,
      FadeTransition(
        opacity: _controller,
        child: AnimatedSize(
          duration: _kToolbarTransitionDuration,
          curve: Curves.decelerate,
          child: GestureDetector(
            onHorizontalDragEnd: _onHorizontalDragEnd,
            child: _CupertinoTextSelectionToolbarItems(
              key: _toolbarItemsKey,
              page: _page,
              backButton: backButton,
              dividerColor: _kToolbarDividerColor.resolveFrom(context),
              dividerWidth: 1.0 / MediaQuery.devicePixelRatioOf(context),
              nextButton: nextButton,
              children: children,
            ),
          ),
        ),
      ),
    );
  }
}

// These classes help to test the chevrons. As _CupertinoChevronPainter must be
// private, it's possible to check the runtimeType of each chevron to know if
// they should be pointing left or right.
class _LeftCupertinoChevronPainter extends _CupertinoChevronPainter {
  _LeftCupertinoChevronPainter({required super.color}) : super(isLeft: true);
}

class _RightCupertinoChevronPainter extends _CupertinoChevronPainter {
  _RightCupertinoChevronPainter({required super.color}) : super(isLeft: false);
}

abstract class _CupertinoChevronPainter extends CustomPainter {
  _CupertinoChevronPainter({required this.color, required this.isLeft});

  final Color color;

  /// If this is true the chevron will point left, else it will point right.
  final bool isLeft;

  @override
  void paint(Canvas canvas, Size size) {
    assert(size.height == size.width, 'size must have the same height and width: $size');

    final double iconSize = size.height;

    // The chevron is half of a square rotated 45˚, so it needs a margin of 1/4
    // its size on each side to be centered horizontally.
    //
    // If pointing left, it means the left half of a square is being used and
    // the offset is positive. If pointing right, the right half is being used
    // and the offset is negative.
    final Offset centerOffset = Offset(iconSize / 4 * (isLeft ? 1 : -1), 0);

    final Offset firstPoint = Offset(iconSize / 2, 0) + centerOffset;
    final Offset middlePoint = Offset(isLeft ? 0 : iconSize, iconSize / 2) + centerOffset;
    final Offset lowerPoint = Offset(iconSize / 2, iconSize) + centerOffset;

    final Paint paint =
        Paint()
          ..color = color
          ..style = PaintingStyle.stroke
          ..strokeWidth = _kToolbarChevronThickness
          ..strokeCap = StrokeCap.round
          ..strokeJoin = StrokeJoin.round;

    // `drawLine` is used here because it's testable. When using `drawPath`,
    // there's no way to test that the chevron points to the correct side.
    canvas.drawLine(firstPoint, middlePoint, paint);
    canvas.drawLine(middlePoint, lowerPoint, paint);
  }

  @override
  bool shouldRepaint(_CupertinoChevronPainter oldDelegate) =>
      oldDelegate.color != color || oldDelegate.isLeft != isLeft;
}

// The custom RenderObjectWidget that, together with
// _RenderCupertinoTextSelectionToolbarItems and
// _CupertinoTextSelectionToolbarItemsElement, paginates the menu items.
class _CupertinoTextSelectionToolbarItems extends RenderObjectWidget {
  _CupertinoTextSelectionToolbarItems({
    super.key,
    required this.page,
    required this.children,
    required this.backButton,
    required this.dividerColor,
    required this.dividerWidth,
    required this.nextButton,
  }) : assert(children.isNotEmpty);

  final Widget backButton;
  final List<Widget> children;
  final Color dividerColor;
  final double dividerWidth;
  final Widget nextButton;
  final int page;

  @override
  _RenderCupertinoTextSelectionToolbarItems createRenderObject(BuildContext context) {
    return _RenderCupertinoTextSelectionToolbarItems(
      dividerColor: dividerColor,
      dividerWidth: dividerWidth,
      page: page,
    );
  }

  @override
  void updateRenderObject(
    BuildContext context,
    _RenderCupertinoTextSelectionToolbarItems renderObject,
  ) {
    renderObject
      ..page = page
      ..dividerColor = dividerColor
      ..dividerWidth = dividerWidth;
  }

  @override
  _CupertinoTextSelectionToolbarItemsElement createElement() =>
      _CupertinoTextSelectionToolbarItemsElement(this);
}

// The custom RenderObjectElement that helps paginate the menu items.
class _CupertinoTextSelectionToolbarItemsElement extends RenderObjectElement {
  _CupertinoTextSelectionToolbarItemsElement(_CupertinoTextSelectionToolbarItems super.widget);

  late List<Element> _children;
  final Map<_CupertinoTextSelectionToolbarItemsSlot, Element> slotToChild =
      <_CupertinoTextSelectionToolbarItemsSlot, Element>{};

  // We keep a set of forgotten children to avoid O(n^2) work walking _children
  // repeatedly to remove children.
  final Set<Element> _forgottenChildren = HashSet<Element>();

  @override
  _RenderCupertinoTextSelectionToolbarItems get renderObject =>
      super.renderObject as _RenderCupertinoTextSelectionToolbarItems;

  void _updateRenderObject(RenderBox? child, _CupertinoTextSelectionToolbarItemsSlot slot) {
    switch (slot) {
      case _CupertinoTextSelectionToolbarItemsSlot.backButton:
        renderObject.backButton = child;
      case _CupertinoTextSelectionToolbarItemsSlot.nextButton:
        renderObject.nextButton = child;
    }
  }

  @override
  void insertRenderObjectChild(RenderObject child, Object? slot) {
    if (slot is _CupertinoTextSelectionToolbarItemsSlot) {
      assert(child is RenderBox);
      _updateRenderObject(child as RenderBox, slot);
      assert(renderObject.slottedChildren.containsKey(slot));
      return;
    }
    if (slot is IndexedSlot) {
      assert(renderObject.debugValidateChild(child));
      renderObject.insert(child as RenderBox, after: slot.value?.renderObject as RenderBox?);
      return;
    }
    assert(false, 'slot must be _CupertinoTextSelectionToolbarItemsSlot or IndexedSlot');
  }

  // This is not reachable for children that don't have an IndexedSlot.
  @override
  void moveRenderObjectChild(
    RenderObject child,
    IndexedSlot<Element> oldSlot,
    IndexedSlot<Element> newSlot,
  ) {
    assert(child.parent == renderObject);
    renderObject.move(child as RenderBox, after: newSlot.value.renderObject as RenderBox?);
  }

  static bool _shouldPaint(Element child) {
    return (child.renderObject!.parentData! as ToolbarItemsParentData).shouldPaint;
  }

  @override
  void removeRenderObjectChild(RenderObject child, Object? slot) {
    // Check if the child is in a slot.
    if (slot is _CupertinoTextSelectionToolbarItemsSlot) {
      assert(child is RenderBox);
      assert(renderObject.slottedChildren.containsKey(slot));
      _updateRenderObject(null, slot);
      assert(!renderObject.slottedChildren.containsKey(slot));
      return;
    }

    // Otherwise look for it in the list of children.
    assert(slot is IndexedSlot);
    assert(child.parent == renderObject);
    renderObject.remove(child as RenderBox);
  }

  @override
  void visitChildren(ElementVisitor visitor) {
    slotToChild.values.forEach(visitor);
    for (final Element child in _children) {
      if (!_forgottenChildren.contains(child)) {
        visitor(child);
      }
    }
  }

  @override
  void forgetChild(Element child) {
    assert(slotToChild.containsValue(child) || _children.contains(child));
    assert(!_forgottenChildren.contains(child));
    // Handle forgetting a child in children or in a slot.
    if (slotToChild.containsKey(child.slot)) {
      final _CupertinoTextSelectionToolbarItemsSlot slot =
          child.slot! as _CupertinoTextSelectionToolbarItemsSlot;
      slotToChild.remove(slot);
    } else {
      _forgottenChildren.add(child);
    }
    super.forgetChild(child);
  }

  // Mount or update slotted child.
  void _mountChild(Widget widget, _CupertinoTextSelectionToolbarItemsSlot slot) {
    final Element? oldChild = slotToChild[slot];
    final Element? newChild = updateChild(oldChild, widget, slot);
    if (oldChild != null) {
      slotToChild.remove(slot);
    }
    if (newChild != null) {
      slotToChild[slot] = newChild;
    }
  }

  @override
  void mount(Element? parent, Object? newSlot) {
    super.mount(parent, newSlot);
    // Mount slotted children.
    final _CupertinoTextSelectionToolbarItems toolbarItems =
        widget as _CupertinoTextSelectionToolbarItems;
    _mountChild(toolbarItems.backButton, _CupertinoTextSelectionToolbarItemsSlot.backButton);
    _mountChild(toolbarItems.nextButton, _CupertinoTextSelectionToolbarItemsSlot.nextButton);

    // Mount list children.
    Element? previousChild;
    _children = List<Element>.generate(toolbarItems.children.length, (int i) {
      final Element result = inflateWidget(
        toolbarItems.children[i],
        IndexedSlot<Element?>(i, previousChild),
      );
      previousChild = result;
      return result;
    }, growable: false);
  }

  @override
  void debugVisitOnstageChildren(ElementVisitor visitor) {
    // Visit slot children.
    for (final Element child in slotToChild.values) {
      if (_shouldPaint(child) && !_forgottenChildren.contains(child)) {
        visitor(child);
      }
    }
    // Visit list children.
    _children
        .where((Element child) => !_forgottenChildren.contains(child) && _shouldPaint(child))
        .forEach(visitor);
  }

  @override
  void update(_CupertinoTextSelectionToolbarItems newWidget) {
    super.update(newWidget);
    assert(widget == newWidget);

    // Update slotted children.
    final _CupertinoTextSelectionToolbarItems toolbarItems =
        widget as _CupertinoTextSelectionToolbarItems;
    _mountChild(toolbarItems.backButton, _CupertinoTextSelectionToolbarItemsSlot.backButton);
    _mountChild(toolbarItems.nextButton, _CupertinoTextSelectionToolbarItemsSlot.nextButton);

    // Update list children.
    _children = updateChildren(
      _children,
      toolbarItems.children,
      forgottenChildren: _forgottenChildren,
    );
    _forgottenChildren.clear();
  }
}

// The custom RenderBox that helps paginate the menu items.
class _RenderCupertinoTextSelectionToolbarItems extends RenderBox
    with
        ContainerRenderObjectMixin<RenderBox, ToolbarItemsParentData>,
        RenderBoxContainerDefaultsMixin<RenderBox, ToolbarItemsParentData> {
  _RenderCupertinoTextSelectionToolbarItems({
    required Color dividerColor,
    required double dividerWidth,
    required int page,
  }) : _dividerColor = dividerColor,
       _dividerWidth = dividerWidth,
       _page = page,
       super();

  final Map<_CupertinoTextSelectionToolbarItemsSlot, RenderBox> slottedChildren =
      <_CupertinoTextSelectionToolbarItemsSlot, RenderBox>{};

  late bool hasNextPage;
  late bool hasPreviousPage;

  RenderBox? _updateChild(
    RenderBox? oldChild,
    RenderBox? newChild,
    _CupertinoTextSelectionToolbarItemsSlot slot,
  ) {
    if (oldChild != null) {
      dropChild(oldChild);
      slottedChildren.remove(slot);
    }
    if (newChild != null) {
      slottedChildren[slot] = newChild;
      adoptChild(newChild);
    }
    return newChild;
  }

  int _page;
  int get page => _page;
  set page(int value) {
    if (value == _page) {
      return;
    }
    _page = value;
    markNeedsLayout();
  }

  Color _dividerColor;
  Color get dividerColor => _dividerColor;
  set dividerColor(Color value) {
    if (value == _dividerColor) {
      return;
    }
    _dividerColor = value;
    markNeedsLayout();
  }

  double _dividerWidth;
  double get dividerWidth => _dividerWidth;
  set dividerWidth(double value) {
    if (value == _dividerWidth) {
      return;
    }
    _dividerWidth = value;
    markNeedsLayout();
  }

  RenderBox? _backButton;
  RenderBox? get backButton => _backButton;
  set backButton(RenderBox? value) {
    _backButton = _updateChild(
      _backButton,
      value,
      _CupertinoTextSelectionToolbarItemsSlot.backButton,
    );
  }

  RenderBox? _nextButton;
  RenderBox? get nextButton => _nextButton;
  set nextButton(RenderBox? value) {
    _nextButton = _updateChild(
      _nextButton,
      value,
      _CupertinoTextSelectionToolbarItemsSlot.nextButton,
    );
  }

  @override
  void performLayout() {
    if (firstChild == null) {
      size = constraints.smallest;
      return;
    }

    // First pass: determine the height of the tallest child.
    double greatestHeight = 0.0;
    visitChildren((RenderObject renderObjectChild) {
      final RenderBox child = renderObjectChild as RenderBox;
      final double childHeight = child.getMaxIntrinsicHeight(constraints.maxWidth);
      if (childHeight > greatestHeight) {
        greatestHeight = childHeight;
      }
    });

    // Layout slotted children.
    final BoxConstraints slottedConstraints = BoxConstraints(
      maxWidth: constraints.maxWidth,
      minHeight: greatestHeight,
      maxHeight: greatestHeight,
    );
    _backButton!.layout(slottedConstraints, parentUsesSize: true);
    _nextButton!.layout(slottedConstraints, parentUsesSize: true);

    final double subsequentPageButtonsWidth = _backButton!.size.width + _nextButton!.size.width;
    double currentButtonPosition = 0.0;
    late double toolbarWidth; // The width of the whole widget.
    int currentPage = 0;
    int i = -1;
    visitChildren((RenderObject renderObjectChild) {
      i++;
      final RenderBox child = renderObjectChild as RenderBox;
      final ToolbarItemsParentData childParentData = child.parentData! as ToolbarItemsParentData;
      childParentData.shouldPaint = false;

      // Skip slotted children and children on pages after the visible page.
      if (child == _backButton || child == _nextButton || currentPage > _page) {
        return;
      }

      // If this is the last child on the first page, it's ok to fit without a forward button.
      // Note childCount doesn't include slotted children which come before the list ones.
      double paginationButtonsWidth =
          currentPage == 0
              ? i == childCount + 1
                  ? 0.0
                  : _nextButton!.size.width
              : subsequentPageButtonsWidth;

      // The width of the menu is set by the first page.
      child.layout(
        BoxConstraints(
          maxWidth: constraints.maxWidth - paginationButtonsWidth,
          minHeight: greatestHeight,
          maxHeight: greatestHeight,
        ),
        parentUsesSize: true,
      );

      // If this child causes the current page to overflow, move to the next
      // page and relayout the child.
      final double currentWidth = currentButtonPosition + paginationButtonsWidth + child.size.width;
      if (currentWidth > constraints.maxWidth) {
        currentPage++;
        currentButtonPosition = _backButton!.size.width + dividerWidth;
        paginationButtonsWidth = _backButton!.size.width + _nextButton!.size.width;
        child.layout(
          BoxConstraints(
            maxWidth: constraints.maxWidth - paginationButtonsWidth,
            minHeight: greatestHeight,
            maxHeight: greatestHeight,
          ),
          parentUsesSize: true,
        );
      }
      childParentData.offset = Offset(currentButtonPosition, 0.0);
      currentButtonPosition += child.size.width + dividerWidth;
      childParentData.shouldPaint = currentPage == page;

      if (currentPage == page) {
        toolbarWidth = currentButtonPosition;
      }
    });

    // It shouldn't be possible to navigate beyond the last page.
    assert(page <= currentPage);

    // Position page nav buttons.
    if (currentPage > 0) {
      final ToolbarItemsParentData nextButtonParentData =
          _nextButton!.parentData! as ToolbarItemsParentData;
      final ToolbarItemsParentData backButtonParentData =
          _backButton!.parentData! as ToolbarItemsParentData;
      // The forward button only shows when there's a page after this one.
      if (page != currentPage) {
        nextButtonParentData.offset = Offset(toolbarWidth, 0.0);
        nextButtonParentData.shouldPaint = true;
        toolbarWidth += nextButton!.size.width;
      }
      if (page > 0) {
        backButtonParentData.offset = Offset.zero;
        backButtonParentData.shouldPaint = true;
        // No need to add the width of the back button to toolbarWidth here. It's
        // already been taken care of when laying out the children to
        // accommodate the back button.
      }
    } else {
      // No divider for the next button when there's only one page.
      toolbarWidth -= dividerWidth;
    }

    // Update previous/next page values so that we can check in the horizontal
    // drag gesture callback if it's possible to navigate.
    hasNextPage = page != currentPage;
    hasPreviousPage = page > 0;

    size = constraints.constrain(Size(toolbarWidth, greatestHeight));
  }

  @override
  void paint(PaintingContext context, Offset offset) {
    visitChildren((RenderObject renderObjectChild) {
      final RenderBox child = renderObjectChild as RenderBox;
      final ToolbarItemsParentData childParentData = child.parentData! as ToolbarItemsParentData;

      if (childParentData.shouldPaint) {
        final Offset childOffset = childParentData.offset + offset;
        context.paintChild(child, childOffset);

        // backButton is a slotted child and is not in the children list, so its
        // childParentData.nextSibling is null. So either when there's a
        // nextSibling or when child is the backButton, draw a divider to the
        // child's right.
        if (childParentData.nextSibling != null || child == backButton) {
          context.canvas.drawLine(
            Offset(child.size.width, 0) + childOffset,
            Offset(child.size.width, child.size.height) + childOffset,
            Paint()..color = dividerColor,
          );
        }
      }
    });
  }

  @override
  void setupParentData(RenderBox child) {
    if (child.parentData is! ToolbarItemsParentData) {
      child.parentData = ToolbarItemsParentData();
    }
  }

  // Returns true if the single child is hit by the given position.
  static bool hitTestChild(RenderBox? child, BoxHitTestResult result, {required Offset position}) {
    if (child == null) {
      return false;
    }
    final ToolbarItemsParentData childParentData = child.parentData! as ToolbarItemsParentData;
    if (!childParentData.shouldPaint) {
      return false;
    }
    return result.addWithPaintOffset(
      offset: childParentData.offset,
      position: position,
      hitTest: (BoxHitTestResult result, Offset transformed) {
        assert(transformed == position - childParentData.offset);
        return child.hitTest(result, position: transformed);
      },
    );
  }

  @override
  bool hitTestChildren(BoxHitTestResult result, {required Offset position}) {
    // Hit test list children.
    RenderBox? child = lastChild;
    while (child != null) {
      final ToolbarItemsParentData childParentData = child.parentData! as ToolbarItemsParentData;

      // Don't hit test children that aren't shown.
      if (!childParentData.shouldPaint) {
        child = childParentData.previousSibling;
        continue;
      }

      if (hitTestChild(child, result, position: position)) {
        return true;
      }
      child = childParentData.previousSibling;
    }

    // Hit test slot children.
    if (hitTestChild(backButton, result, position: position)) {
      return true;
    }
    if (hitTestChild(nextButton, result, position: position)) {
      return true;
    }

    return false;
  }

  @override
  void attach(PipelineOwner owner) {
    // Attach list children.
    super.attach(owner);

    // Attach slot children.
    for (final RenderBox child in slottedChildren.values) {
      child.attach(owner);
    }
  }

  @override
  void detach() {
    // Detach list children.
    super.detach();

    // Detach slot children.
    for (final RenderBox child in slottedChildren.values) {
      child.detach();
    }
  }

  @override
  void redepthChildren() {
    visitChildren((RenderObject renderObjectChild) {
      final RenderBox child = renderObjectChild as RenderBox;
      redepthChild(child);
    });
  }

  @override
  void visitChildren(RenderObjectVisitor visitor) {
    // Visit the slotted children.
    if (_backButton != null) {
      visitor(_backButton!);
    }
    if (_nextButton != null) {
      visitor(_nextButton!);
    }
    // Visit the list children.
    super.visitChildren(visitor);
  }

  // Visit only the children that should be painted.
  @override
  void visitChildrenForSemantics(RenderObjectVisitor visitor) {
    visitChildren((RenderObject renderObjectChild) {
      final RenderBox child = renderObjectChild as RenderBox;
      final ToolbarItemsParentData childParentData = child.parentData! as ToolbarItemsParentData;
      if (childParentData.shouldPaint) {
        visitor(renderObjectChild);
      }
    });
  }

  @override
  List<DiagnosticsNode> debugDescribeChildren() {
    final List<DiagnosticsNode> value = <DiagnosticsNode>[];
    visitChildren((RenderObject renderObjectChild) {
      final RenderBox child = renderObjectChild as RenderBox;
      if (child == backButton) {
        value.add(child.toDiagnosticsNode(name: 'back button'));
      } else if (child == nextButton) {
        value.add(child.toDiagnosticsNode(name: 'next button'));

        // List children.
      } else {
        value.add(child.toDiagnosticsNode(name: 'menu item'));
      }
    });
    return value;
  }
}

// The slots that can be occupied by widgets in
// _CupertinoTextSelectionToolbarItems, excluding the list of children.
enum _CupertinoTextSelectionToolbarItemsSlot { backButton, nextButton }
