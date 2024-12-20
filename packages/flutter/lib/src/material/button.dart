// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'button_style_button.dart';
/// @docImport 'elevated_button.dart';
/// @docImport 'filled_button.dart';
/// @docImport 'material_button.dart';
/// @docImport 'outlined_button.dart';
/// @docImport 'text_button.dart';
library;

import 'dart:math' as math;

import 'package:flutter/foundation.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/widgets.dart';

import 'button_theme.dart';
import 'constants.dart';
import 'ink_well.dart';
import 'material.dart';
import 'material_state.dart';
import 'material_state_mixin.dart';
import 'theme.dart';
import 'theme_data.dart';

/// Creates a button based on [Semantics], [Material], and [InkWell]
/// widgets.
///
/// This class does not use the current [Theme] or [ButtonTheme] to
/// compute default values for unspecified parameters. It's intended to
/// be used for custom Material buttons that optionally incorporate defaults
/// from the themes or from app-specific sources.
///
/// This class is planned to be deprecated in a future release, see
/// [ButtonStyleButton], the base class of [ElevatedButton], [FilledButton],
/// [OutlinedButton] and [TextButton].
///
/// See also:
///
///  * [ElevatedButton], a filled button whose material elevates when pressed.
///  * [FilledButton], a filled button that doesn't elevate when pressed.
///  * [FilledButton.tonal], a filled button variant that uses a secondary fill color.
///  * [OutlinedButton], a button with an outlined border and no fill color.
///  * [TextButton], a button with no outline or fill color.
@Category(<String>['Material', 'Button'])
class RawMaterialButton extends StatefulWidget {
  /// Create a button based on [Semantics], [Material], and [InkWell] widgets.
  ///
  /// The [elevation], [focusElevation], [hoverElevation], [highlightElevation],
  /// and [disabledElevation] parameters must be non-negative.
  const RawMaterialButton({
    super.key,
    required this.onPressed,
    this.onLongPress,
    this.onHighlightChanged,
    this.mouseCursor,
    this.textStyle,
    this.fillColor,
    this.focusColor,
    this.hoverColor,
    this.highlightColor,
    this.splashColor,
    this.elevation = 2.0,
    this.focusElevation = 4.0,
    this.hoverElevation = 4.0,
    this.highlightElevation = 8.0,
    this.disabledElevation = 0.0,
    this.padding = EdgeInsets.zero,
    this.visualDensity = VisualDensity.standard,
    this.constraints = const BoxConstraints(minWidth: 88.0, minHeight: 36.0),
    this.shape = const RoundedRectangleBorder(),
    this.animationDuration = kThemeChangeDuration,
    this.clipBehavior = Clip.none,
    this.focusNode,
    this.autofocus = false,
    MaterialTapTargetSize? materialTapTargetSize,
    this.child,
    this.enableFeedback = true,
  }) : materialTapTargetSize = materialTapTargetSize ?? MaterialTapTargetSize.padded,
       assert(elevation >= 0.0),
       assert(focusElevation >= 0.0),
       assert(hoverElevation >= 0.0),
       assert(highlightElevation >= 0.0),
       assert(disabledElevation >= 0.0);

  /// Called when the button is tapped or otherwise activated.
  ///
  /// If this callback and [onLongPress] are null, then the button will be disabled.
  ///
  /// See also:
  ///
  ///  * [enabled], which is true if the button is enabled.
  final VoidCallback? onPressed;

  /// Called when the button is long-pressed.
  ///
  /// If this callback and [onPressed] are null, then the button will be disabled.
  ///
  /// See also:
  ///
  ///  * [enabled], which is true if the button is enabled.
  final VoidCallback? onLongPress;

  /// Called by the underlying [InkWell] widget's [InkWell.onHighlightChanged]
  /// callback.
  ///
  /// If [onPressed] changes from null to non-null while a gesture is ongoing,
  /// this can fire during the build phase (in which case calling
  /// [State.setState] is not allowed).
  final ValueChanged<bool>? onHighlightChanged;

  /// {@template flutter.material.RawMaterialButton.mouseCursor}
  /// The cursor for a mouse pointer when it enters or is hovering over the
  /// button.
  ///
  /// If [mouseCursor] is a [WidgetStateMouseCursor],
  /// [WidgetStateProperty.resolve] is used for the following [WidgetState]s:
  ///
  ///  * [WidgetState.pressed].
  ///  * [WidgetState.hovered].
  ///  * [WidgetState.focused].
  ///  * [WidgetState.disabled].
  /// {@endtemplate}
  ///
  /// If this property is null, [WidgetStateMouseCursor.clickable] will be used.
  final MouseCursor? mouseCursor;

  /// Defines the default text style, with [Material.textStyle], for the
  /// button's [child].
  ///
  /// If [TextStyle.color] is a [WidgetStateProperty<Color>], [WidgetStateProperty.resolve]
  /// is used for the following [WidgetState]s:
  ///
  ///  * [WidgetState.pressed].
  ///  * [WidgetState.hovered].
  ///  * [WidgetState.focused].
  ///  * [WidgetState.disabled].
  final TextStyle? textStyle;

  /// The color of the button's [Material].
  final Color? fillColor;

  /// The color for the button's [Material] when it has the input focus.
  final Color? focusColor;

  /// The color for the button's [Material] when a pointer is hovering over it.
  final Color? hoverColor;

  /// The highlight color for the button's [InkWell].
  final Color? highlightColor;

  /// The splash color for the button's [InkWell].
  final Color? splashColor;

  /// The elevation for the button's [Material] when the button
  /// is [enabled] but not pressed.
  ///
  /// Defaults to 2.0. The value is always non-negative.
  ///
  /// See also:
  ///
  ///  * [highlightElevation], the default elevation.
  ///  * [hoverElevation], the elevation when a pointer is hovering over the
  ///    button.
  ///  * [focusElevation], the elevation when the button is focused.
  ///  * [disabledElevation], the elevation when the button is disabled.
  final double elevation;

  /// The elevation for the button's [Material] when the button
  /// is [enabled] and a pointer is hovering over it.
  ///
  /// Defaults to 4.0. The value is always non-negative.
  ///
  /// If the button is [enabled], and being pressed (in the highlighted state),
  /// then the [highlightElevation] take precedence over the [hoverElevation].
  ///
  /// See also:
  ///
  ///  * [elevation], the default elevation.
  ///  * [focusElevation], the elevation when the button is focused.
  ///  * [disabledElevation], the elevation when the button is disabled.
  ///  * [highlightElevation], the elevation when the button is pressed.
  final double hoverElevation;

  /// The elevation for the button's [Material] when the button
  /// is [enabled] and has the input focus.
  ///
  /// Defaults to 4.0. The value is always non-negative.
  ///
  /// If the button is [enabled], and being pressed (in the highlighted state),
  /// or a mouse cursor is hovering over the button, then the [hoverElevation]
  /// and [highlightElevation] take precedence over the [focusElevation].
  ///
  /// See also:
  ///
  ///  * [elevation], the default elevation.
  ///  * [hoverElevation], the elevation when a pointer is hovering over the
  ///    button.
  ///  * [disabledElevation], the elevation when the button is disabled.
  ///  * [highlightElevation], the elevation when the button is pressed.
  final double focusElevation;

  /// The elevation for the button's [Material] when the button
  /// is [enabled] and pressed.
  ///
  /// Defaults to 8.0. The value is always non-negative.
  ///
  /// See also:
  ///
  ///  * [elevation], the default elevation.
  ///  * [hoverElevation], the elevation when a pointer is hovering over the
  ///    button.
  ///  * [focusElevation], the elevation when the button is focused.
  ///  * [disabledElevation], the elevation when the button is disabled.
  final double highlightElevation;

  /// The elevation for the button's [Material] when the button
  /// is not [enabled].
  ///
  /// Defaults to 0.0. The value is always non-negative.
  ///
  /// See also:
  ///
  ///  * [elevation], the default elevation.
  ///  * [hoverElevation], the elevation when a pointer is hovering over the
  ///    button.
  ///  * [focusElevation], the elevation when the button is focused.
  ///  * [highlightElevation], the elevation when the button is pressed.
  final double disabledElevation;

  /// The internal padding for the button's [child].
  final EdgeInsetsGeometry padding;

  /// Defines how compact the button's layout will be.
  ///
  /// {@macro flutter.material.themedata.visualDensity}
  ///
  /// See also:
  ///
  ///  * [ThemeData.visualDensity], which specifies the [visualDensity] for all widgets
  ///    within a [Theme].
  final VisualDensity visualDensity;

  /// Defines the button's size.
  ///
  /// Typically used to constrain the button's minimum size.
  final BoxConstraints constraints;

  /// The shape of the button's [Material].
  ///
  /// The button's highlight and splash are clipped to this shape. If the
  /// button has an elevation, then its drop shadow is defined by this shape.
  ///
  /// If [shape] is a [WidgetStateProperty<ShapeBorder>], [WidgetStateProperty.resolve]
  /// is used for the following [WidgetState]s:
  ///
  /// * [WidgetState.pressed].
  /// * [WidgetState.hovered].
  /// * [WidgetState.focused].
  /// * [WidgetState.disabled].
  final ShapeBorder shape;

  /// Defines the duration of animated changes for [shape] and [elevation].
  ///
  /// The default value is [kThemeChangeDuration].
  final Duration animationDuration;

  /// Typically the button's label.
  final Widget? child;

  /// Whether the button is enabled or disabled.
  ///
  /// Buttons are disabled by default. To enable a button, set its [onPressed]
  /// or [onLongPress] properties to a non-null value.
  bool get enabled => onPressed != null || onLongPress != null;

  /// Configures the minimum size of the tap target.
  ///
  /// Defaults to [MaterialTapTargetSize.padded].
  ///
  /// See also:
  ///
  ///  * [MaterialTapTargetSize], for a description of how this affects tap targets.
  final MaterialTapTargetSize materialTapTargetSize;

  /// {@macro flutter.widgets.Focus.focusNode}
  final FocusNode? focusNode;

  /// {@macro flutter.widgets.Focus.autofocus}
  final bool autofocus;

  /// {@macro flutter.material.Material.clipBehavior}
  ///
  /// Defaults to [Clip.none].
  final Clip clipBehavior;

  /// Whether detected gestures should provide acoustic and/or haptic feedback.
  ///
  /// For example, on Android a tap will produce a clicking sound and a
  /// long-press will produce a short vibration, when feedback is enabled.
  ///
  /// See also:
  ///
  ///  * [Feedback] for providing platform-specific feedback to certain actions.
  final bool enableFeedback;

  @override
  State<RawMaterialButton> createState() => _RawMaterialButtonState();
}

class _RawMaterialButtonState extends State<RawMaterialButton> with MaterialStateMixin {
  @override
  void initState() {
    super.initState();
    setMaterialState(MaterialState.disabled, !widget.enabled);
  }

  @override
  void didUpdateWidget(RawMaterialButton oldWidget) {
    super.didUpdateWidget(oldWidget);
    setMaterialState(MaterialState.disabled, !widget.enabled);
    // If the button is disabled while a press gesture is currently ongoing,
    // InkWell makes a call to handleHighlightChanged. This causes an exception
    // because it calls setState in the middle of a build. To preempt this, we
    // manually update pressed to false when this situation occurs.
    if (isDisabled && isPressed) {
      removeMaterialState(MaterialState.pressed);
    }
  }

  double get _effectiveElevation {
    // These conditionals are in order of precedence, so be careful about
    // reorganizing them.
    if (isDisabled) {
      return widget.disabledElevation;
    }
    if (isPressed) {
      return widget.highlightElevation;
    }
    if (isHovered) {
      return widget.hoverElevation;
    }
    if (isFocused) {
      return widget.focusElevation;
    }
    return widget.elevation;
  }

  @override
  Widget build(BuildContext context) {
    final Color? effectiveTextColor = MaterialStateProperty.resolveAs<Color?>(
      widget.textStyle?.color,
      materialStates,
    );
    final ShapeBorder? effectiveShape = MaterialStateProperty.resolveAs<ShapeBorder?>(
      widget.shape,
      materialStates,
    );
    final Offset densityAdjustment = widget.visualDensity.baseSizeAdjustment;
    final BoxConstraints effectiveConstraints = widget.visualDensity.effectiveConstraints(
      widget.constraints,
    );
    final MouseCursor? effectiveMouseCursor = MaterialStateProperty.resolveAs<MouseCursor?>(
      widget.mouseCursor ?? MaterialStateMouseCursor.clickable,
      materialStates,
    );
    final EdgeInsetsGeometry padding = widget.padding
        .add(
          EdgeInsets.only(
            left: densityAdjustment.dx,
            top: densityAdjustment.dy,
            right: densityAdjustment.dx,
            bottom: densityAdjustment.dy,
          ),
        )
        .clamp(EdgeInsets.zero, EdgeInsetsGeometry.infinity);

    final Widget result = ConstrainedBox(
      constraints: effectiveConstraints,
      child: Material(
        elevation: _effectiveElevation,
        textStyle: widget.textStyle?.copyWith(color: effectiveTextColor),
        shape: effectiveShape,
        color: widget.fillColor,
        // For compatibility during the M3 migration the default shadow needs to be passed.
        shadowColor: Theme.of(context).useMaterial3 ? Theme.of(context).shadowColor : null,
        type: widget.fillColor == null ? MaterialType.transparency : MaterialType.button,
        animationDuration: widget.animationDuration,
        clipBehavior: widget.clipBehavior,
        child: InkWell(
          focusNode: widget.focusNode,
          canRequestFocus: widget.enabled,
          onFocusChange: updateMaterialState(MaterialState.focused),
          autofocus: widget.autofocus,
          onHighlightChanged: updateMaterialState(
            MaterialState.pressed,
            onChanged: widget.onHighlightChanged,
          ),
          splashColor: widget.splashColor,
          highlightColor: widget.highlightColor,
          focusColor: widget.focusColor,
          hoverColor: widget.hoverColor,
          onHover: updateMaterialState(MaterialState.hovered),
          onTap: widget.onPressed,
          onLongPress: widget.onLongPress,
          enableFeedback: widget.enableFeedback,
          customBorder: effectiveShape,
          mouseCursor: effectiveMouseCursor,
          child: IconTheme.merge(
            data: IconThemeData(color: effectiveTextColor),
            child: Padding(
              padding: padding,
              child: Center(widthFactor: 1.0, heightFactor: 1.0, child: widget.child),
            ),
          ),
        ),
      ),
    );
    final Size minSize;
    switch (widget.materialTapTargetSize) {
      case MaterialTapTargetSize.padded:
        minSize = Size(
          kMinInteractiveDimension + densityAdjustment.dx,
          kMinInteractiveDimension + densityAdjustment.dy,
        );
        assert(minSize.width >= 0.0);
        assert(minSize.height >= 0.0);
      case MaterialTapTargetSize.shrinkWrap:
        minSize = Size.zero;
    }

    return Semantics(
      container: true,
      button: true,
      enabled: widget.enabled,
      child: _InputPadding(minSize: minSize, child: result),
    );
  }
}

/// A widget to pad the area around a [MaterialButton]'s inner [Material].
///
/// Redirect taps that occur in the padded area around the child to the center
/// of the child. This increases the size of the button and the button's
/// "tap target", but not its material or its ink splashes.
class _InputPadding extends SingleChildRenderObjectWidget {
  const _InputPadding({super.child, required this.minSize});

  final Size minSize;

  @override
  RenderObject createRenderObject(BuildContext context) {
    return _RenderInputPadding(minSize);
  }

  @override
  void updateRenderObject(BuildContext context, covariant _RenderInputPadding renderObject) {
    renderObject.minSize = minSize;
  }
}

class _RenderInputPadding extends RenderShiftedBox {
  _RenderInputPadding(this._minSize, [RenderBox? child]) : super(child);

  Size get minSize => _minSize;
  Size _minSize;
  set minSize(Size value) {
    if (_minSize == value) {
      return;
    }
    _minSize = value;
    markNeedsLayout();
  }

  @override
  double computeMinIntrinsicWidth(double height) {
    if (child != null) {
      return math.max(child!.getMinIntrinsicWidth(height), minSize.width);
    }
    return 0.0;
  }

  @override
  double computeMinIntrinsicHeight(double width) {
    if (child != null) {
      return math.max(child!.getMinIntrinsicHeight(width), minSize.height);
    }
    return 0.0;
  }

  @override
  double computeMaxIntrinsicWidth(double height) {
    if (child != null) {
      return math.max(child!.getMaxIntrinsicWidth(height), minSize.width);
    }
    return 0.0;
  }

  @override
  double computeMaxIntrinsicHeight(double width) {
    if (child != null) {
      return math.max(child!.getMaxIntrinsicHeight(width), minSize.height);
    }
    return 0.0;
  }

  Size _computeSize({required BoxConstraints constraints, required ChildLayouter layoutChild}) {
    if (child != null) {
      final Size childSize = layoutChild(child!, constraints);
      final double height = math.max(childSize.width, minSize.width);
      final double width = math.max(childSize.height, minSize.height);
      return constraints.constrain(Size(height, width));
    }
    return Size.zero;
  }

  @override
  Size computeDryLayout(BoxConstraints constraints) {
    return _computeSize(constraints: constraints, layoutChild: ChildLayoutHelper.dryLayoutChild);
  }

  @override
  double? computeDryBaseline(covariant BoxConstraints constraints, TextBaseline baseline) {
    final RenderBox? child = this.child;
    if (child == null) {
      return null;
    }
    final double? result = child.getDryBaseline(constraints, baseline);
    if (result == null) {
      return null;
    }
    final Size childSize = child.getDryLayout(constraints);
    return result +
        Alignment.center.alongOffset(getDryLayout(constraints) - childSize as Offset).dy;
  }

  @override
  void performLayout() {
    size = _computeSize(constraints: constraints, layoutChild: ChildLayoutHelper.layoutChild);
    if (child != null) {
      final BoxParentData childParentData = child!.parentData! as BoxParentData;
      childParentData.offset = Alignment.center.alongOffset(size - child!.size as Offset);
    }
  }

  @override
  bool hitTest(BoxHitTestResult result, {required Offset position}) {
    if (super.hitTest(result, position: position)) {
      return true;
    }
    final Offset center = child!.size.center(Offset.zero);
    return result.addWithRawTransform(
      transform: MatrixUtils.forceToPoint(center),
      position: center,
      hitTest: (BoxHitTestResult result, Offset position) {
        assert(position == center);
        return child!.hitTest(result, position: center);
      },
    );
  }
}
