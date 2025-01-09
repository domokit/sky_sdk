// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'card.dart';
/// @docImport 'dropdown.dart';
/// @docImport 'elevated_button.dart';
/// @docImport 'outlined_button.dart';
/// @docImport 'text_button.dart';
/// @docImport 'theme_data.dart';
library;

import 'package:flutter/rendering.dart';
import 'package:flutter/widgets.dart';

import 'button_bar_theme.dart';
import 'button_theme.dart';
import 'dialog.dart';

/// An end-aligned row of buttons, laying out into a column if there is not
/// enough horizontal space.
///
/// ## Updating to [OverflowBar]
///
/// [ButtonBar] has been replace by a more efficient widget, [OverflowBar].
///
/// ```dart
/// // Before
/// ButtonBar(
///   alignment: MainAxisAlignment.spaceEvenly,
///   children: <Widget>[
///     TextButton( child: const Text('Button 1'), onPressed: () {}),
///     TextButton( child: const Text('Button 2'), onPressed: () {}),
///     TextButton( child: const Text('Button 3'), onPressed: () {}),
///   ],
/// );
/// ```
/// ```dart
/// // After
/// OverflowBar(
///   alignment: MainAxisAlignment.spaceEvenly,
///   children: <Widget>[
///     TextButton( child: const Text('Button 1'), onPressed: () {}),
///     TextButton( child: const Text('Button 2'), onPressed: () {}),
///     TextButton( child: const Text('Button 3'), onPressed: () {}),
///   ],
/// );
/// ```
///
/// See the [OverflowBar] documentation for more details.
///
/// ## Using [ButtonBar]
///
/// Places the buttons horizontally according to the [buttonPadding]. The
/// children are laid out in a [Row] with [MainAxisAlignment.end]. When the
/// [Directionality] is [TextDirection.ltr], the button bar's children are
/// right justified and the last child becomes the rightmost child. When the
/// [Directionality] [TextDirection.rtl] the children are left justified and
/// the last child becomes the leftmost child.
///
/// If the button bar's width exceeds the maximum width constraint on the
/// widget, it aligns its buttons in a column. The key difference here
/// is that the [MainAxisAlignment] will then be treated as a
/// cross-axis/horizontal alignment. For example, if the buttons overflow and
/// [ButtonBar.alignment] was set to [MainAxisAlignment.start], the buttons would
/// align to the horizontal start of the button bar.
///
/// The [ButtonBar] can be configured with a [ButtonBarTheme]. For any null
/// property on the ButtonBar, the surrounding ButtonBarTheme's property
/// will be used instead. If the ButtonBarTheme's property is null
/// as well, the property will default to a value described in the field
/// documentation below.
///
/// The [children] are wrapped in a [ButtonTheme] that is a copy of the
/// surrounding ButtonTheme with the button properties overridden by the
/// properties of the ButtonBar as described above. These properties include
/// [buttonTextTheme], [buttonMinWidth], [buttonHeight], [buttonPadding],
/// and [buttonAlignedDropdown].
///
/// Used by [Dialog] to arrange the actions at the bottom of the dialog.
///
/// See also:
///
///  * [TextButton], a simple flat button without a shadow.
///  * [ElevatedButton], a filled button whose material elevates when pressed.
///  * [OutlinedButton], a [TextButton] with a border outline.
///  * [Card], at the bottom of which it is common to place a [ButtonBar].
///  * [Dialog], which uses a [ButtonBar] for its actions.
///  * [ButtonBarTheme], which configures the [ButtonBar].
@Deprecated(
  'Use OverflowBar instead. '
  'This feature was deprecated after v3.21.0-10.0.pre.',
)
class ButtonBar extends StatelessWidget {
  /// Creates a button bar.
  ///
  /// Both [buttonMinWidth] and [buttonHeight] must be non-negative if they
  /// are not null.
  @Deprecated(
    'Use OverflowBar instead. '
    'This feature was deprecated after v3.21.0-10.0.pre.',
  )
  const ButtonBar({
    super.key,
    this.alignment,
    this.mainAxisSize,
    this.buttonTextTheme,
    this.buttonMinWidth,
    this.buttonHeight,
    this.buttonPadding,
    this.buttonAlignedDropdown,
    this.layoutBehavior,
    this.overflowDirection,
    this.overflowButtonSpacing,
    this.children = const <Widget>[],
  }) : assert(buttonMinWidth == null || buttonMinWidth >= 0.0),
       assert(buttonHeight == null || buttonHeight >= 0.0),
       assert(overflowButtonSpacing == null || overflowButtonSpacing >= 0.0);

  /// How the children should be placed along the horizontal axis.
  ///
  /// If null then it will use [ButtonBarThemeData.alignment]. If that is null,
  /// it will default to [MainAxisAlignment.end].
  final MainAxisAlignment? alignment;

  /// How much horizontal space is available. See [Row.mainAxisSize].
  ///
  /// If null then it will use the surrounding [ButtonBarThemeData.mainAxisSize].
  /// If that is null, it will default to [MainAxisSize.max].
  final MainAxisSize? mainAxisSize;

  /// Overrides the surrounding [ButtonBarThemeData.buttonTextTheme] to define a
  /// button's base colors, size, internal padding and shape.
  ///
  /// If null then it will use the surrounding
  /// [ButtonBarThemeData.buttonTextTheme]. If that is null, it will default to
  /// [ButtonTextTheme.primary].
  final ButtonTextTheme? buttonTextTheme;

  /// Overrides the surrounding [ButtonThemeData.minWidth] to define a button's
  /// minimum width.
  ///
  /// If null then it will use the surrounding [ButtonBarThemeData.buttonMinWidth].
  /// If that is null, it will default to 64.0 logical pixels.
  final double? buttonMinWidth;

  /// Overrides the surrounding [ButtonThemeData.height] to define a button's
  /// minimum height.
  ///
  /// If null then it will use the surrounding [ButtonBarThemeData.buttonHeight].
  /// If that is null, it will default to 36.0 logical pixels.
  final double? buttonHeight;

  /// Overrides the surrounding [ButtonThemeData.padding] to define the padding
  /// for a button's child (typically the button's label).
  ///
  /// If null then it will use the surrounding [ButtonBarThemeData.buttonPadding].
  /// If that is null, it will default to 8.0 logical pixels on the left
  /// and right.
  final EdgeInsetsGeometry? buttonPadding;

  /// Overrides the surrounding [ButtonThemeData.alignedDropdown] to define whether
  /// a [DropdownButton] menu's width will match the button's width.
  ///
  /// If null then it will use the surrounding [ButtonBarThemeData.buttonAlignedDropdown].
  /// If that is null, it will default to false.
  final bool? buttonAlignedDropdown;

  /// Defines whether a [ButtonBar] should size itself with a minimum size
  /// constraint or with padding.
  ///
  /// Overrides the surrounding [ButtonThemeData.layoutBehavior].
  ///
  /// If null then it will use the surrounding [ButtonBarThemeData.layoutBehavior].
  /// If that is null, it will default [ButtonBarLayoutBehavior.padded].
  final ButtonBarLayoutBehavior? layoutBehavior;

  /// Defines the vertical direction of a [ButtonBar]'s children if it
  /// overflows.
  ///
  /// If [children] do not fit into a single row, then they
  /// are arranged in a column. The first action is at the top of the
  /// column if this property is set to [VerticalDirection.down], since it
  /// "starts" at the top and "ends" at the bottom. On the other hand,
  /// the first action will be at the bottom of the column if this
  /// property is set to [VerticalDirection.up], since it "starts" at the
  /// bottom and "ends" at the top.
  ///
  /// If null then it will use the surrounding
  /// [ButtonBarThemeData.overflowDirection]. If that is null, it will
  /// default to [VerticalDirection.down].
  final VerticalDirection? overflowDirection;

  /// The spacing between buttons when the button bar overflows.
  ///
  /// If the [children] do not fit into a single row, they are arranged into a
  /// column. This parameter provides additional vertical space in between
  /// buttons when it does overflow.
  ///
  /// The button spacing may appear to be more than the value provided. This is
  /// because most buttons adhere to the [MaterialTapTargetSize] of 48px. So,
  /// even though a button might visually be 36px in height, it might still take
  /// up to 48px vertically.
  ///
  /// If null then no spacing will be added in between buttons in
  /// an overflow state.
  final double? overflowButtonSpacing;

  /// The buttons to arrange horizontally.
  ///
  /// Typically [ElevatedButton] or [TextButton] widgets.
  final List<Widget> children;

  @override
  Widget build(BuildContext context) {
    final ButtonThemeData parentButtonTheme = ButtonTheme.of(context);
    final ButtonBarThemeData barTheme = ButtonBarTheme.of(context);

    final ButtonThemeData buttonTheme = parentButtonTheme.copyWith(
      textTheme: buttonTextTheme ?? barTheme.buttonTextTheme ?? ButtonTextTheme.primary,
      minWidth: buttonMinWidth ?? barTheme.buttonMinWidth ?? 64.0,
      height: buttonHeight ?? barTheme.buttonHeight ?? 36.0,
      padding:
          buttonPadding ?? barTheme.buttonPadding ?? const EdgeInsets.symmetric(horizontal: 8.0),
      alignedDropdown: buttonAlignedDropdown ?? barTheme.buttonAlignedDropdown ?? false,
      layoutBehavior: layoutBehavior ?? barTheme.layoutBehavior ?? ButtonBarLayoutBehavior.padded,
    );

    // We divide by 4.0 because we want half of the average of the left and right padding.
    final double paddingUnit = buttonTheme.padding.horizontal / 4.0;
    final Widget child = ButtonTheme.fromButtonThemeData(
      data: buttonTheme,
      child: _ButtonBarRow(
        mainAxisAlignment: alignment ?? barTheme.alignment ?? MainAxisAlignment.end,
        mainAxisSize: mainAxisSize ?? barTheme.mainAxisSize ?? MainAxisSize.max,
        overflowDirection:
            overflowDirection ?? barTheme.overflowDirection ?? VerticalDirection.down,
        overflowButtonSpacing: overflowButtonSpacing,
        children:
            children.map<Widget>((Widget child) {
              return Padding(padding: EdgeInsets.symmetric(horizontal: paddingUnit), child: child);
            }).toList(),
      ),
    );
    switch (buttonTheme.layoutBehavior) {
      case ButtonBarLayoutBehavior.padded:
        return Padding(
          padding: EdgeInsets.symmetric(vertical: 2.0 * paddingUnit, horizontal: paddingUnit),
          child: child,
        );
      case ButtonBarLayoutBehavior.constrained:
        return ConstrainedBox(
          constraints: const BoxConstraints(minHeight: 52.0),
          child: Padding(
            padding: EdgeInsets.symmetric(horizontal: paddingUnit),
            child: Center(child: child),
          ),
        );
    }
  }
}

/// Attempts to display buttons in a row, but displays them in a column if
/// there is not enough horizontal space.
///
/// It first attempts to lay out its buttons as though there were no
/// maximum width constraints on the widget. If the button bar's width is
/// less than the maximum width constraints of the widget, it then lays
/// out the widget as though it were placed in a [Row].
///
/// However, if the button bar's width exceeds the maximum width constraint on
/// the widget, it then aligns its buttons in a column. The key difference here
/// is that the [MainAxisAlignment] will then be treated as a
/// cross-axis/horizontal alignment. For example, if the buttons overflow and
/// [ButtonBar.alignment] was set to [MainAxisAlignment.start], the column of
/// buttons would align to the horizontal start of the button bar.
class _ButtonBarRow extends Flex {
  /// Creates a button bar that attempts to display in a row, but displays in
  /// a column if there is insufficient horizontal space.
  const _ButtonBarRow({
    required super.children,
    super.mainAxisSize,
    super.mainAxisAlignment,
    VerticalDirection overflowDirection = VerticalDirection.down,
    this.overflowButtonSpacing,
  }) : super(direction: Axis.horizontal, verticalDirection: overflowDirection);

  final double? overflowButtonSpacing;

  @override
  _RenderButtonBarRow createRenderObject(BuildContext context) {
    return _RenderButtonBarRow(
      direction: direction,
      mainAxisAlignment: mainAxisAlignment,
      mainAxisSize: mainAxisSize,
      crossAxisAlignment: crossAxisAlignment,
      textDirection: getEffectiveTextDirection(context)!,
      verticalDirection: verticalDirection,
      textBaseline: textBaseline,
      overflowButtonSpacing: overflowButtonSpacing,
    );
  }

  @override
  void updateRenderObject(BuildContext context, covariant _RenderButtonBarRow renderObject) {
    renderObject
      ..direction = direction
      ..mainAxisAlignment = mainAxisAlignment
      ..mainAxisSize = mainAxisSize
      ..crossAxisAlignment = crossAxisAlignment
      ..textDirection = getEffectiveTextDirection(context)
      ..verticalDirection = verticalDirection
      ..textBaseline = textBaseline
      ..overflowButtonSpacing = overflowButtonSpacing;
  }
}

/// Attempts to display buttons in a row, but displays them in a column if
/// there is not enough horizontal space.
///
/// It first attempts to lay out its buttons as though there were no
/// maximum width constraints on the widget. If the button bar's width is
/// less than the maximum width constraints of the widget, it then lays
/// out the widget as though it were placed in a [Row].
///
/// However, if the button bar's width exceeds the maximum width constraint on
/// the widget, it then aligns its buttons in a column. The key difference here
/// is that the [MainAxisAlignment] will then be treated as a
/// cross-axis/horizontal alignment. For example, if the buttons overflow and
/// [ButtonBar.alignment] was set to [MainAxisAlignment.start], the buttons would
/// align to the horizontal start of the button bar.
class _RenderButtonBarRow extends RenderFlex {
  /// Creates a button bar that attempts to display in a row, but displays in
  /// a column if there is insufficient horizontal space.
  _RenderButtonBarRow({
    super.direction,
    super.mainAxisSize,
    super.mainAxisAlignment,
    super.crossAxisAlignment,
    required TextDirection super.textDirection,
    super.verticalDirection,
    super.textBaseline,
    this.overflowButtonSpacing,
  }) : assert(overflowButtonSpacing == null || overflowButtonSpacing >= 0);

  bool _hasCheckedLayoutWidth = false;
  double? overflowButtonSpacing;

  @override
  BoxConstraints get constraints {
    if (_hasCheckedLayoutWidth) {
      return super.constraints;
    }
    return super.constraints.copyWith(maxWidth: double.infinity);
  }

  @override
  Size computeDryLayout(BoxConstraints constraints) {
    final Size size = super.computeDryLayout(constraints.copyWith(maxWidth: double.infinity));
    if (size.width <= constraints.maxWidth) {
      return super.computeDryLayout(constraints);
    }
    double currentHeight = 0.0;
    RenderBox? child = firstChild;
    while (child != null) {
      final BoxConstraints childConstraints = constraints.copyWith(minWidth: 0.0);
      final Size childSize = child.getDryLayout(childConstraints);
      currentHeight += childSize.height;
      child = childAfter(child);
      if (overflowButtonSpacing != null && child != null) {
        currentHeight += overflowButtonSpacing!;
      }
    }
    return constraints.constrain(Size(constraints.maxWidth, currentHeight));
  }

  @override
  void performLayout() {
    // Set check layout width to false in reload or update cases.
    _hasCheckedLayoutWidth = false;

    // Perform layout to ensure that button bar knows how wide it would
    // ideally want to be.
    super.performLayout();
    _hasCheckedLayoutWidth = true;

    // If the button bar is constrained by width and it overflows, set the
    // buttons to align vertically. Otherwise, lay out the button bar
    // horizontally.
    if (size.width <= constraints.maxWidth) {
      // A second performLayout is required to ensure that the original maximum
      // width constraints are used. The original perform layout call assumes
      // a maximum width constraint of infinity.
      super.performLayout();
    } else {
      final BoxConstraints childConstraints = constraints.copyWith(minWidth: 0.0);
      double currentHeight = 0.0;
      RenderBox? child = switch (verticalDirection) {
        VerticalDirection.down => firstChild,
        VerticalDirection.up => lastChild,
      };

      while (child != null) {
        final FlexParentData childParentData = child.parentData! as FlexParentData;

        // Lay out the child with the button bar's original constraints, but
        // with minimum width set to zero.
        child.layout(childConstraints, parentUsesSize: true);

        // Set the cross axis alignment for the column to match the main axis
        // alignment for a row. For [MainAxisAlignment.spaceAround],
        // [MainAxisAlignment.spaceBetween] and [MainAxisAlignment.spaceEvenly]
        // cases, use [MainAxisAlignment.start].
        switch (textDirection!) {
          case TextDirection.ltr:
            switch (mainAxisAlignment) {
              case MainAxisAlignment.center:
                final double midpoint = (constraints.maxWidth - child.size.width) / 2.0;
                childParentData.offset = Offset(midpoint, currentHeight);
              case MainAxisAlignment.end:
                childParentData.offset = Offset(
                  constraints.maxWidth - child.size.width,
                  currentHeight,
                );
              case MainAxisAlignment.spaceAround:
              case MainAxisAlignment.spaceBetween:
              case MainAxisAlignment.spaceEvenly:
              case MainAxisAlignment.start:
                childParentData.offset = Offset(0, currentHeight);
            }
          case TextDirection.rtl:
            switch (mainAxisAlignment) {
              case MainAxisAlignment.center:
                final double midpoint = constraints.maxWidth / 2.0 - child.size.width / 2.0;
                childParentData.offset = Offset(midpoint, currentHeight);
              case MainAxisAlignment.end:
                childParentData.offset = Offset(0, currentHeight);
              case MainAxisAlignment.spaceAround:
              case MainAxisAlignment.spaceBetween:
              case MainAxisAlignment.spaceEvenly:
              case MainAxisAlignment.start:
                childParentData.offset = Offset(
                  constraints.maxWidth - child.size.width,
                  currentHeight,
                );
            }
        }
        currentHeight += child.size.height;
        child = switch (verticalDirection) {
          VerticalDirection.down => childParentData.nextSibling,
          VerticalDirection.up => childParentData.previousSibling,
        };

        if (overflowButtonSpacing != null && child != null) {
          currentHeight += overflowButtonSpacing!;
        }
      }
      size = constraints.constrain(Size(constraints.maxWidth, currentHeight));
    }
  }
}
