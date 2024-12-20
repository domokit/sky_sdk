// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'icon_button.dart';
/// @docImport 'navigation_rail.dart';
/// @docImport 'text_button.dart';
/// @docImport 'text_theme.dart';
library;

import 'dart:math' as math;

import 'package:flutter/rendering.dart';
import 'package:flutter/widgets.dart';

import 'badge_theme.dart';
import 'color_scheme.dart';
import 'theme.dart';

/// A Material Design "badge".
///
/// A badge's [label] conveys a small amount of information about its
/// [child], like a count or status. If the label is null then this is
/// a "small" badge that's displayed as a [smallSize] diameter filled
/// circle. Otherwise this is a [StadiumBorder] shaped "large" badge
/// with height [largeSize].
///
/// Badges are typically used to decorate the icon within a
/// [BottomNavigationBarItem] or a [NavigationRailDestination]
/// or a button's icon, as in [TextButton.icon]. The badge's default
/// configuration is intended to work well with a default sized (24)
/// [Icon].
///
/// {@tool dartpad}
/// This example shows how to create a [Badge] with label and count
/// wrapped on an icon in an [IconButton].
///
/// ** See code in examples/api/lib/material/badge/badge.0.dart **
/// {@end-tool}
class Badge extends StatelessWidget {
  /// Create a Badge that stacks [label] on top of [child].
  ///
  /// If [label] is null then just a filled circle is displayed. Otherwise
  /// the [label] is displayed within a [StadiumBorder] shaped area.
  const Badge({
    super.key,
    this.backgroundColor,
    this.textColor,
    this.smallSize,
    this.largeSize,
    this.textStyle,
    this.padding,
    this.alignment,
    this.offset,
    this.label,
    this.isLabelVisible = true,
    this.child,
  });

  /// Convenience constructor for creating a badge with a numeric
  /// label with 1-3 digits based on [count].
  ///
  /// Initializes [label] with a [Text] widget that contains [count].
  /// If [count] is greater than 999, then the label is '999+'.
  Badge.count({
    super.key,
    this.backgroundColor,
    this.textColor,
    this.smallSize,
    this.largeSize,
    this.textStyle,
    this.padding,
    this.alignment,
    this.offset,
    required int count,
    this.isLabelVisible = true,
    this.child,
  }) : label = Text(count > 999 ? '999+' : '$count');

  /// The badge's fill color.
  ///
  /// Defaults to the [BadgeTheme]'s background color, or
  /// [ColorScheme.error] if the theme value is null.
  final Color? backgroundColor;

  /// The color of the badge's [label] text.
  ///
  /// This color overrides the color of the label's [textStyle].
  ///
  /// Defaults to the [BadgeTheme]'s foreground color, or
  /// [ColorScheme.onError] if the theme value is null.
  final Color? textColor;

  /// The diameter of the badge if [label] is null.
  ///
  /// Defaults to the [BadgeTheme]'s small size, or 6 if the theme value
  /// is null.
  final double? smallSize;

  /// The badge's height if [label] is non-null.
  ///
  /// Defaults to the [BadgeTheme]'s large size, or 16 if the theme value
  /// is null. If the default value is overridden then it may be useful to
  /// also override [padding] and [alignment].
  final double? largeSize;

  /// The [DefaultTextStyle] for the badge's label.
  ///
  /// The text style's color is overwritten by the [textColor].
  ///
  /// This value is only used if [label] is non-null.
  ///
  /// Defaults to the [BadgeTheme]'s text style, or the overall theme's
  /// [TextTheme.labelSmall] if the badge theme's value is null. If
  /// the default text style is overridden then it may be useful to
  /// also override [largeSize], [padding], and [alignment].
  final TextStyle? textStyle;

  /// The padding added to the badge's label.
  ///
  /// This value is only used if [label] is non-null.
  ///
  /// Defaults to the [BadgeTheme]'s padding, or 4 pixels on the
  /// left and right if the theme's value is null.
  final EdgeInsetsGeometry? padding;

  /// Combined with [offset] to determine the location of the [label]
  /// relative to the [child].
  ///
  /// The alignment positions the label in the same way a child of an
  /// [Align] widget is positioned, except that, the alignment is
  /// resolved as if the label was a [largeSize] square and [offset]
  /// is added to the result.
  ///
  /// This value is only used if [label] is non-null.
  ///
  /// Defaults to the [BadgeTheme]'s alignment, or
  /// [AlignmentDirectional.topEnd] if the theme's value is null.
  final AlignmentGeometry? alignment;

  /// Combined with [alignment] to determine the location of the [label]
  /// relative to the [child].
  ///
  /// This value is only used if [label] is non-null.
  ///
  /// Defaults to the [BadgeTheme]'s offset, or
  /// if the theme's value is null then `Offset(4, -4)` for
  /// [TextDirection.ltr] or `Offset(-4, -4)` for [TextDirection.rtl].
  final Offset? offset;

  /// The badge's content, typically a [Text] widget that contains 1 to 4
  /// characters.
  ///
  /// If the label is null then this is a "small" badge that's
  /// displayed as a [smallSize] diameter filled circle. Otherwise
  /// this is a [StadiumBorder] shaped "large" badge with height [largeSize].
  final Widget? label;

  /// If false, the badge's [label] is not included.
  ///
  /// This flag is true by default. It's intended to make it convenient
  /// to create a badge that's only shown under certain conditions.
  final bool isLabelVisible;

  /// The widget that the badge is stacked on top of.
  ///
  /// Typically this is an default sized [Icon] that's part of a
  /// [BottomNavigationBarItem] or a [NavigationRailDestination].
  final Widget? child;

  @override
  Widget build(BuildContext context) {
    if (!isLabelVisible) {
      return child ?? const SizedBox();
    }

    final BadgeThemeData badgeTheme = BadgeTheme.of(context);
    final BadgeThemeData defaults = _BadgeDefaultsM3(context);
    final Decoration effectiveDecoration = ShapeDecoration(
      color: backgroundColor ?? badgeTheme.backgroundColor ?? defaults.backgroundColor!,
      shape: const StadiumBorder(),
    );
    final double effectiveWidthOffset;
    final Widget badge;
    final bool hasLabel = label != null;
    if (hasLabel) {
      final double minSize =
          effectiveWidthOffset = largeSize ?? badgeTheme.largeSize ?? defaults.largeSize!;
      badge = DefaultTextStyle(
        style: (textStyle ?? badgeTheme.textStyle ?? defaults.textStyle!).copyWith(
          color: textColor ?? badgeTheme.textColor ?? defaults.textColor!,
        ),
        child: _IntrinsicHorizontalStadium(
          minSize: minSize,
          child: Container(
            clipBehavior: Clip.antiAlias,
            decoration: effectiveDecoration,
            padding: padding ?? badgeTheme.padding ?? defaults.padding!,
            alignment: Alignment.center,
            child: label,
          ),
        ),
      );
    } else {
      final double effectiveSmallSize =
          effectiveWidthOffset = smallSize ?? badgeTheme.smallSize ?? defaults.smallSize!;
      badge = Container(
        width: effectiveSmallSize,
        height: effectiveSmallSize,
        clipBehavior: Clip.antiAlias,
        decoration: effectiveDecoration,
      );
    }

    if (child == null) {
      return badge;
    }

    final AlignmentGeometry effectiveAlignment =
        alignment ?? badgeTheme.alignment ?? defaults.alignment!;
    final TextDirection textDirection = Directionality.of(context);
    final Offset defaultOffset =
        textDirection == TextDirection.ltr ? const Offset(4, -4) : const Offset(-4, -4);
    // Adds a offset const Offset(0, 8) to avoiding breaking customers after
    // the offset calculation changes.
    // See https://github.com/flutter/flutter/pull/146853.
    final Offset effectiveOffset =
        (offset ?? badgeTheme.offset ?? defaultOffset) + const Offset(0, 8);

    return Stack(
      clipBehavior: Clip.none,
      children: <Widget>[
        child!,
        Positioned.fill(
          child: _Badge(
            alignment: effectiveAlignment,
            offset: hasLabel ? effectiveOffset : Offset.zero,
            hasLabel: hasLabel,
            widthOffset: effectiveWidthOffset,
            textDirection: textDirection,
            child: badge,
          ),
        ),
      ],
    );
  }
}

class _Badge extends SingleChildRenderObjectWidget {
  const _Badge({
    required this.alignment,
    required this.offset,
    required this.widthOffset,
    required this.textDirection,
    required this.hasLabel,
    super.child, // the badge
  });

  final AlignmentGeometry alignment;
  final Offset offset;
  final double widthOffset;
  final TextDirection textDirection;
  final bool hasLabel;

  @override
  _RenderBadge createRenderObject(BuildContext context) {
    return _RenderBadge(
      alignment: alignment,
      widthOffset: widthOffset,
      hasLabel: hasLabel,
      offset: offset,
      textDirection: Directionality.maybeOf(context),
    );
  }

  @override
  void updateRenderObject(BuildContext context, _RenderBadge renderObject) {
    renderObject
      ..alignment = alignment
      ..offset = offset
      ..widthOffset = widthOffset
      ..hasLabel = hasLabel
      ..textDirection = Directionality.maybeOf(context);
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DiagnosticsProperty<AlignmentGeometry>('alignment', alignment));
    properties.add(DiagnosticsProperty<Offset>('offset', offset));
  }
}

class _RenderBadge extends RenderAligningShiftedBox {
  _RenderBadge({
    super.textDirection,
    super.alignment,
    required Offset offset,
    required bool hasLabel,
    required double widthOffset,
  }) : _offset = offset,
       _hasLabel = hasLabel,
       _widthOffset = widthOffset;

  Offset get offset => _offset;
  Offset _offset;
  set offset(Offset value) {
    if (_offset == value) {
      return;
    }
    _offset = value;
    markNeedsLayout();
  }

  bool get hasLabel => _hasLabel;
  bool _hasLabel;
  set hasLabel(bool value) {
    if (_hasLabel == value) {
      return;
    }
    _hasLabel = value;
    markNeedsLayout();
  }

  double get widthOffset => _widthOffset;
  double _widthOffset;
  set widthOffset(double value) {
    if (_widthOffset == value) {
      return;
    }
    _widthOffset = value;
    markNeedsLayout();
  }

  @override
  void performLayout() {
    final BoxConstraints constraints = this.constraints;
    assert(constraints.hasBoundedWidth);
    assert(constraints.hasBoundedHeight);
    size = constraints.biggest;

    child!.layout(const BoxConstraints(), parentUsesSize: true);
    final double badgeSize = child!.size.height;
    final Alignment resolvedAlignment = alignment.resolve(textDirection);
    final BoxParentData childParentData = child!.parentData! as BoxParentData;
    Offset badgeLocation =
        offset + resolvedAlignment.alongOffset(Offset(size.width - widthOffset, size.height));
    if (hasLabel) {
      // Adjust for label height.
      badgeLocation = badgeLocation - Offset(0, badgeSize / 2);
    }
    childParentData.offset = badgeLocation;
  }
}

/// A widget size itself to the smallest horizontal stadium rect that can still
/// fit the child's intrinsic size.
///
/// A horizontal stadium means a rect that has width >= height.
///
/// Uses [minSize] to set the min size of width and height.
class _IntrinsicHorizontalStadium extends SingleChildRenderObjectWidget {
  const _IntrinsicHorizontalStadium({super.child, required this.minSize});
  final double minSize;

  @override
  _RenderIntrinsicHorizontalStadium createRenderObject(BuildContext context) {
    return _RenderIntrinsicHorizontalStadium(minSize: minSize);
  }
}

class _RenderIntrinsicHorizontalStadium extends RenderProxyBox {
  _RenderIntrinsicHorizontalStadium({RenderBox? child, required double minSize})
    : _minSize = minSize,
      super(child);

  double get minSize => _minSize;
  double _minSize;
  set minSize(double value) {
    if (_minSize == value) {
      return;
    }
    _minSize = value;
    markNeedsLayout();
  }

  @override
  double computeMinIntrinsicWidth(double height) {
    return getMaxIntrinsicWidth(height);
  }

  @override
  double computeMaxIntrinsicWidth(double height) {
    return math.max(getMaxIntrinsicHeight(double.infinity), super.computeMaxIntrinsicWidth(height));
  }

  @override
  double computeMinIntrinsicHeight(double width) {
    return getMaxIntrinsicHeight(width);
  }

  @override
  double computeMaxIntrinsicHeight(double width) {
    return math.max(minSize, super.computeMaxIntrinsicHeight(width));
  }

  BoxConstraints _childConstraints(RenderBox child, BoxConstraints constraints) {
    final double childHeight = math.max(minSize, child.getMaxIntrinsicHeight(constraints.maxWidth));
    final double childWidth = child.getMaxIntrinsicWidth(constraints.maxHeight);
    return constraints.tighten(width: math.max(childWidth, childHeight), height: childHeight);
  }

  Size _computeSize({required ChildLayouter layoutChild, required BoxConstraints constraints}) {
    final RenderBox child = this.child!;
    final Size childSize = layoutChild(child, _childConstraints(child, constraints));
    if (childSize.height > childSize.width) {
      return Size(childSize.height, childSize.height);
    }
    return childSize;
  }

  @override
  @protected
  Size computeDryLayout(covariant BoxConstraints constraints) {
    return _computeSize(layoutChild: ChildLayoutHelper.dryLayoutChild, constraints: constraints);
  }

  @override
  double? computeDryBaseline(BoxConstraints constraints, TextBaseline baseline) {
    final RenderBox child = this.child!;
    return child.getDryBaseline(_childConstraints(child, constraints), baseline);
  }

  @override
  void performLayout() {
    size = _computeSize(layoutChild: ChildLayoutHelper.layoutChild, constraints: constraints);
  }
}

// BEGIN GENERATED TOKEN PROPERTIES - Badge

// Do not edit by hand. The code between the "BEGIN GENERATED" and
// "END GENERATED" comments are generated from data in the Material
// Design token database by the script:
//   dev/tools/gen_defaults/bin/gen_defaults.dart.

// dart format off
class _BadgeDefaultsM3 extends BadgeThemeData {
  _BadgeDefaultsM3(this.context) : super(
    smallSize: 6.0,
    largeSize: 16.0,
    padding: const EdgeInsets.symmetric(horizontal: 4),
    alignment: AlignmentDirectional.topEnd,
  );

  final BuildContext context;
  late final ThemeData _theme = Theme.of(context);
  late final ColorScheme _colors = _theme.colorScheme;

  @override
  Color? get backgroundColor => _colors.error;

  @override
  Color? get textColor => _colors.onError;

  @override
  TextStyle? get textStyle => Theme.of(context).textTheme.labelSmall;
}
// dart format on

// END GENERATED TOKEN PROPERTIES - Badge
