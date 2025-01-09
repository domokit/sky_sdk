// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'data_table.dart';
/// @docImport 'elevated_button.dart';
/// @docImport 'icon_button.dart';
/// @docImport 'ink_decoration.dart';
/// @docImport 'ink_ripple.dart';
/// @docImport 'ink_splash.dart';
/// @docImport 'text_button.dart';
library;

import 'dart:async';
import 'dart:collection';

import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/widgets.dart';

import 'debug.dart';
import 'ink_highlight.dart';
import 'material.dart';
import 'material_state.dart';
import 'theme.dart';

// Examples can assume:
// late BuildContext context;

/// An ink feature that displays a [color] "splash" in response to a user
/// gesture that can be confirmed or canceled.
///
/// Subclasses call [confirm] when an input gesture is recognized. For
/// example a press event might trigger an ink feature that's confirmed
/// when the corresponding up event is seen.
///
/// Subclasses call [cancel] when an input gesture is aborted before it
/// is recognized. For example a press event might trigger an ink feature
/// that's canceled when the pointer is dragged out of the reference
/// box.
///
/// The [InkWell] and [InkResponse] widgets generate instances of this
/// class.
abstract class InteractiveInkFeature extends InkFeature {
  /// Creates an InteractiveInkFeature.
  InteractiveInkFeature({
    required super.controller,
    required super.referenceBox,
    required Color color,
    ShapeBorder? customBorder,
    super.onRemoved,
  }) : _color = color,
       _customBorder = customBorder;

  /// Called when the user input that triggered this feature's appearance was confirmed.
  ///
  /// Typically causes the ink to propagate faster across the material. By default this
  /// method does nothing.
  void confirm() {}

  /// Called when the user input that triggered this feature's appearance was canceled.
  ///
  /// Typically causes the ink to gradually disappear. By default this method does
  /// nothing.
  void cancel() {}

  /// The ink's color.
  Color get color => _color;
  Color _color;
  set color(Color value) {
    if (value == _color) {
      return;
    }
    _color = value;
    controller.markNeedsPaint();
  }

  /// The ink's optional custom border.
  ShapeBorder? get customBorder => _customBorder;
  ShapeBorder? _customBorder;
  set customBorder(ShapeBorder? value) {
    if (value == _customBorder) {
      return;
    }
    _customBorder = value;
    controller.markNeedsPaint();
  }

  /// Draws an ink splash or ink ripple on the passed in [Canvas].
  ///
  /// The [transform] argument is the [Matrix4] transform that typically
  /// shifts the coordinate space of the canvas to the space in which
  /// the ink circle is to be painted.
  ///
  /// [center] is the [Offset] from origin of the canvas where the center
  /// of the circle is drawn.
  ///
  /// [paint] takes a [Paint] object that describes the styles used to draw the ink circle.
  /// For example, [paint] can specify properties like color, strokewidth, colorFilter.
  ///
  /// [radius] is the radius of ink circle to be drawn on canvas.
  ///
  /// [clipCallback] is the callback used to obtain the [Rect] used for clipping the ink effect.
  /// If [clipCallback] is null, no clipping is performed on the ink circle.
  ///
  /// Clipping can happen in 3 different ways:
  ///  1. If [customBorder] is provided, it is used to determine the path
  ///     for clipping.
  ///  2. If [customBorder] is null, and [borderRadius] is provided, the canvas
  ///     is clipped by an [RRect] created from [clipCallback] and [borderRadius].
  ///  3. If [borderRadius] is the default [BorderRadius.zero], then the [Rect] provided
  ///      by [clipCallback] is used for clipping.
  ///
  /// [textDirection] is used by [customBorder] if it is non-null. This allows the [customBorder]'s path
  /// to be properly defined if it was the path was expressed in terms of "start" and "end" instead of
  /// "left" and "right".
  ///
  /// For examples on how the function is used, see [InkSplash] and [InkRipple].
  @protected
  void paintInkCircle({
    required Canvas canvas,
    required Matrix4 transform,
    required Paint paint,
    required Offset center,
    required double radius,
    TextDirection? textDirection,
    ShapeBorder? customBorder,
    BorderRadius borderRadius = BorderRadius.zero,
    RectCallback? clipCallback,
  }) {
    final Offset? originOffset = MatrixUtils.getAsTranslation(transform);
    canvas.save();
    if (originOffset == null) {
      canvas.transform(transform.storage);
    } else {
      canvas.translate(originOffset.dx, originOffset.dy);
    }
    if (clipCallback != null) {
      final Rect rect = clipCallback();
      if (customBorder != null) {
        canvas.clipPath(customBorder.getOuterPath(rect, textDirection: textDirection));
      } else if (borderRadius != BorderRadius.zero) {
        canvas.clipRRect(
          RRect.fromRectAndCorners(
            rect,
            topLeft: borderRadius.topLeft,
            topRight: borderRadius.topRight,
            bottomLeft: borderRadius.bottomLeft,
            bottomRight: borderRadius.bottomRight,
          ),
        );
      } else {
        canvas.clipRect(rect);
      }
    }
    canvas.drawCircle(center, radius, paint);
    canvas.restore();
  }
}

/// An encapsulation of an [InteractiveInkFeature] constructor used by
/// [InkWell], [InkResponse], and [ThemeData].
///
/// Interactive ink feature implementations should provide a static const
/// `splashFactory` value that's an instance of this class. The `splashFactory`
/// can be used to configure an [InkWell], [InkResponse] or [ThemeData].
///
/// See also:
///
///  * [InkSplash.splashFactory]
///  * [InkRipple.splashFactory]
abstract class InteractiveInkFeatureFactory {
  /// Abstract const constructor. This constructor enables subclasses to provide
  /// const constructors so that they can be used in const expressions.
  ///
  /// Subclasses should provide a const constructor.
  const InteractiveInkFeatureFactory();

  /// The factory method.
  ///
  /// Subclasses should override this method to return a new instance of an
  /// [InteractiveInkFeature].
  @factory
  InteractiveInkFeature create({
    required MaterialInkController controller,
    required RenderBox referenceBox,
    required Offset position,
    required Color color,
    required TextDirection textDirection,
    bool containedInkWell = false,
    RectCallback? rectCallback,
    BorderRadius? borderRadius,
    ShapeBorder? customBorder,
    double? radius,
    VoidCallback? onRemoved,
  });
}

abstract class _ParentInkResponseState {
  void markChildInkResponsePressed(_ParentInkResponseState childState, bool value);
}

class _ParentInkResponseProvider extends InheritedWidget {
  const _ParentInkResponseProvider({required this.state, required super.child});

  final _ParentInkResponseState state;

  @override
  bool updateShouldNotify(_ParentInkResponseProvider oldWidget) => state != oldWidget.state;

  static _ParentInkResponseState? maybeOf(BuildContext context) {
    return context.dependOnInheritedWidgetOfExactType<_ParentInkResponseProvider>()?.state;
  }
}

typedef _GetRectCallback = RectCallback? Function(RenderBox referenceBox);
typedef _CheckContext = bool Function(BuildContext context);

/// An area of a [Material] that responds to touch. Has a configurable shape and
/// can be configured to clip splashes that extend outside its bounds or not.
///
/// For a variant of this widget that is specialized for rectangular areas that
/// always clip splashes, see [InkWell].
///
/// An [InkResponse] widget does two things when responding to a tap:
///
///  * It starts to animate a _highlight_. The shape of the highlight is
///    determined by [highlightShape]. If it is a [BoxShape.circle], the
///    default, then the highlight is a circle of fixed size centered in the
///    [InkResponse]. If it is [BoxShape.rectangle], then the highlight is a box
///    the size of the [InkResponse] itself, unless [getRectCallback] is
///    provided, in which case that callback defines the rectangle. The color of
///    the highlight is set by [highlightColor].
///
///  * Simultaneously, it starts to animate a _splash_. This is a growing circle
///    initially centered on the tap location. If this is a [containedInkWell],
///    the splash grows to the [radius] while remaining centered at the tap
///    location. Otherwise, the splash migrates to the center of the box as it
///    grows.
///
/// The following two diagrams show how [InkResponse] looks when tapped if the
/// [highlightShape] is [BoxShape.circle] (the default) and [containedInkWell]
/// is false (also the default).
///
/// The first diagram shows how it looks if the [InkResponse] is relatively
/// large:
///
/// ![The highlight is a disc centered in the box, smaller than the child widget.](https://flutter.github.io/assets-for-api-docs/assets/material/ink_response_large.png)
///
/// The second diagram shows how it looks if the [InkResponse] is small:
///
/// ![The highlight is a disc overflowing the box, centered on the child.](https://flutter.github.io/assets-for-api-docs/assets/material/ink_response_small.png)
///
/// The main thing to notice from these diagrams is that the splashes happily
/// exceed the bounds of the widget (because [containedInkWell] is false).
///
/// The following diagram shows the effect when the [InkResponse] has a
/// [highlightShape] of [BoxShape.rectangle] with [containedInkWell] set to
/// true. These are the values used by [InkWell].
///
/// ![The highlight is a rectangle the size of the box.](https://flutter.github.io/assets-for-api-docs/assets/material/ink_well.png)
///
/// The [InkResponse] widget must have a [Material] widget as an ancestor. The
/// [Material] widget is where the ink reactions are actually painted. This
/// matches the Material Design premise wherein the [Material] is what is
/// actually reacting to touches by spreading ink.
///
/// If a Widget uses this class directly, it should include the following line
/// at the top of its build function to call [debugCheckHasMaterial]:
///
/// ```dart
/// assert(debugCheckHasMaterial(context));
/// ```
///
/// ## Troubleshooting
///
/// ### The ink splashes aren't visible!
///
/// If there is an opaque graphic, e.g. painted using a [Container], [Image], or
/// [DecoratedBox], between the [Material] widget and the [InkResponse] widget,
/// then the splash won't be visible because it will be under the opaque graphic.
/// This is because ink splashes draw on the underlying [Material] itself, as
/// if the ink was spreading inside the material.
///
/// The [Ink] widget can be used as a replacement for [Image], [Container], or
/// [DecoratedBox] to ensure that the image or decoration also paints in the
/// [Material] itself, below the ink.
///
/// If this is not possible for some reason, e.g. because you are using an
/// opaque [CustomPaint] widget, alternatively consider using a second
/// [Material] above the opaque widget but below the [InkResponse] (as an
/// ancestor to the ink response). The [MaterialType.transparency] material
/// kind can be used for this purpose.
///
/// See also:
///
///  * [GestureDetector], for listening for gestures without ink splashes.
///  * [ElevatedButton] and [TextButton], two kinds of buttons in Material Design.
///  * [IconButton], which combines [InkResponse] with an [Icon].
class InkResponse extends StatelessWidget {
  /// Creates an area of a [Material] that responds to touch.
  ///
  /// Must have an ancestor [Material] widget in which to cause ink reactions.
  const InkResponse({
    super.key,
    this.child,
    this.onTap,
    this.onTapDown,
    this.onTapUp,
    this.onTapCancel,
    this.onDoubleTap,
    this.onLongPress,
    this.onSecondaryTap,
    this.onSecondaryTapUp,
    this.onSecondaryTapDown,
    this.onSecondaryTapCancel,
    this.onHighlightChanged,
    this.onHover,
    this.mouseCursor,
    this.containedInkWell = false,
    this.highlightShape = BoxShape.circle,
    this.radius,
    this.borderRadius,
    this.customBorder,
    this.focusColor,
    this.hoverColor,
    this.highlightColor,
    this.overlayColor,
    this.splashColor,
    this.splashFactory,
    this.enableFeedback = true,
    this.excludeFromSemantics = false,
    this.focusNode,
    this.canRequestFocus = true,
    this.onFocusChange,
    this.autofocus = false,
    this.statesController,
    this.hoverDuration,
  });

  /// The widget below this widget in the tree.
  ///
  /// {@macro flutter.widgets.ProxyWidget.child}
  final Widget? child;

  /// Called when the user taps this part of the material.
  final GestureTapCallback? onTap;

  /// Called when the user taps down this part of the material.
  final GestureTapDownCallback? onTapDown;

  /// Called when the user releases a tap that was started on this part of the
  /// material. [onTap] is called immediately after.
  final GestureTapUpCallback? onTapUp;

  /// Called when the user cancels a tap that was started on this part of the
  /// material.
  final GestureTapCallback? onTapCancel;

  /// Called when the user double taps this part of the material.
  final GestureTapCallback? onDoubleTap;

  /// Called when the user long-presses on this part of the material.
  final GestureLongPressCallback? onLongPress;

  /// Called when the user taps this part of the material with a secondary button.
  final GestureTapCallback? onSecondaryTap;

  /// Called when the user taps down on this part of the material with a
  /// secondary button.
  final GestureTapDownCallback? onSecondaryTapDown;

  /// Called when the user releases a secondary button tap that was started on
  /// this part of the material. [onSecondaryTap] is called immediately after.
  final GestureTapUpCallback? onSecondaryTapUp;

  /// Called when the user cancels a secondary button tap that was started on
  /// this part of the material.
  final GestureTapCallback? onSecondaryTapCancel;

  /// Called when this part of the material either becomes highlighted or stops
  /// being highlighted.
  ///
  /// The value passed to the callback is true if this part of the material has
  /// become highlighted and false if this part of the material has stopped
  /// being highlighted.
  ///
  /// If all of [onTap], [onDoubleTap], and [onLongPress] become null while a
  /// gesture is ongoing, then [onTapCancel] will be fired and
  /// [onHighlightChanged] will be fired with the value false _during the
  /// build_. This means, for instance, that in that scenario [State.setState]
  /// cannot be called.
  final ValueChanged<bool>? onHighlightChanged;

  /// Called when a pointer enters or exits the ink response area.
  ///
  /// The value passed to the callback is true if a pointer has entered this
  /// part of the material and false if a pointer has exited this part of the
  /// material.
  final ValueChanged<bool>? onHover;

  /// The cursor for a mouse pointer when it enters or is hovering over the
  /// widget.
  ///
  /// If [mouseCursor] is a [WidgetStateMouseCursor],
  /// [WidgetStateProperty.resolve] is used for the following [WidgetState]s:
  ///
  ///  * [WidgetState.hovered].
  ///  * [WidgetState.focused].
  ///  * [WidgetState.disabled].
  ///
  /// If this property is null, [WidgetStateMouseCursor.clickable] will be used.
  final MouseCursor? mouseCursor;

  /// Whether this ink response should be clipped its bounds.
  ///
  /// This flag also controls whether the splash migrates to the center of the
  /// [InkResponse] or not. If [containedInkWell] is true, the splash remains
  /// centered around the tap location. If it is false, the splash migrates to
  /// the center of the [InkResponse] as it grows.
  ///
  /// See also:
  ///
  ///  * [highlightShape], the shape of the focus, hover, and pressed
  ///    highlights.
  ///  * [borderRadius], which controls the corners when the box is a rectangle.
  ///  * [getRectCallback], which controls the size and position of the box when
  ///    it is a rectangle.
  final bool containedInkWell;

  /// The shape (e.g., circle, rectangle) to use for the highlight drawn around
  /// this part of the material when pressed, hovered over, or focused.
  ///
  /// The same shape is used for the pressed highlight (see [highlightColor]),
  /// the focus highlight (see [focusColor]), and the hover highlight (see
  /// [hoverColor]).
  ///
  /// If the shape is [BoxShape.circle], then the highlight is centered on the
  /// [InkResponse]. If the shape is [BoxShape.rectangle], then the highlight
  /// fills the [InkResponse], or the rectangle provided by [getRectCallback] if
  /// the callback is specified.
  ///
  /// See also:
  ///
  ///  * [containedInkWell], which controls clipping behavior.
  ///  * [borderRadius], which controls the corners when the box is a rectangle.
  ///  * [highlightColor], the color of the highlight.
  ///  * [getRectCallback], which controls the size and position of the box when
  ///    it is a rectangle.
  final BoxShape highlightShape;

  /// The radius of the ink splash.
  ///
  /// Splashes grow up to this size. By default, this size is determined from
  /// the size of the rectangle provided by [getRectCallback], or the size of
  /// the [InkResponse] itself.
  ///
  /// See also:
  ///
  ///  * [splashColor], the color of the splash.
  ///  * [splashFactory], which defines the appearance of the splash.
  final double? radius;

  /// The border radius of the containing rectangle. This is effective only if
  /// [highlightShape] is [BoxShape.rectangle].
  ///
  /// If this is null, it is interpreted as [BorderRadius.zero].
  final BorderRadius? borderRadius;

  /// The custom clip border.
  ///
  /// If this is null, the ink response will not clip its content.
  final ShapeBorder? customBorder;

  /// The color of the ink response when the parent widget is focused. If this
  /// property is null then the focus color of the theme,
  /// [ThemeData.focusColor], will be used.
  ///
  /// See also:
  ///
  ///  * [highlightShape], the shape of the focus, hover, and pressed
  ///    highlights.
  ///  * [hoverColor], the color of the hover highlight.
  ///  * [splashColor], the color of the splash.
  ///  * [splashFactory], which defines the appearance of the splash.
  final Color? focusColor;

  /// The color of the ink response when a pointer is hovering over it. If this
  /// property is null then the hover color of the theme,
  /// [ThemeData.hoverColor], will be used.
  ///
  /// See also:
  ///
  ///  * [highlightShape], the shape of the focus, hover, and pressed
  ///    highlights.
  ///  * [highlightColor], the color of the pressed highlight.
  ///  * [focusColor], the color of the focus highlight.
  ///  * [splashColor], the color of the splash.
  ///  * [splashFactory], which defines the appearance of the splash.
  final Color? hoverColor;

  /// The highlight color of the ink response when pressed. If this property is
  /// null then the highlight color of the theme, [ThemeData.highlightColor],
  /// will be used.
  ///
  /// See also:
  ///
  ///  * [hoverColor], the color of the hover highlight.
  ///  * [focusColor], the color of the focus highlight.
  ///  * [highlightShape], the shape of the focus, hover, and pressed
  ///    highlights.
  ///  * [splashColor], the color of the splash.
  ///  * [splashFactory], which defines the appearance of the splash.
  final Color? highlightColor;

  /// Defines the ink response focus, hover, and splash colors.
  ///
  /// This default null property can be used as an alternative to
  /// [focusColor], [hoverColor], [highlightColor], and
  /// [splashColor]. If non-null, it is resolved against one of
  /// [WidgetState.focused], [WidgetState.hovered], and
  /// [WidgetState.pressed]. It's convenient to use when the parent
  /// widget can pass along its own WidgetStateProperty value for
  /// the overlay color.
  ///
  /// [WidgetState.pressed] triggers a ripple (an ink splash), per
  /// the current Material Design spec. The [overlayColor] doesn't map
  /// a state to [highlightColor] because a separate highlight is not
  /// used by the current design guidelines. See
  /// https://material.io/design/interaction/states.html#pressed
  ///
  /// If the overlay color is null or resolves to null, then [focusColor],
  /// [hoverColor], [splashColor] and their defaults are used instead.
  ///
  /// See also:
  ///
  ///  * The Material Design specification for overlay colors and how they
  ///    match a component's state:
  ///    <https://material.io/design/interaction/states.html#anatomy>.
  final MaterialStateProperty<Color?>? overlayColor;

  /// The splash color of the ink response. If this property is null then the
  /// splash color of the theme, [ThemeData.splashColor], will be used.
  ///
  /// See also:
  ///
  ///  * [splashFactory], which defines the appearance of the splash.
  ///  * [radius], the (maximum) size of the ink splash.
  ///  * [highlightColor], the color of the highlight.
  final Color? splashColor;

  /// Defines the appearance of the splash.
  ///
  /// Defaults to the value of the theme's splash factory: [ThemeData.splashFactory].
  ///
  /// See also:
  ///
  ///  * [radius], the (maximum) size of the ink splash.
  ///  * [splashColor], the color of the splash.
  ///  * [highlightColor], the color of the highlight.
  ///  * [InkSplash.splashFactory], which defines the default splash.
  ///  * [InkRipple.splashFactory], which defines a splash that spreads out
  ///    more aggressively than the default.
  final InteractiveInkFeatureFactory? splashFactory;

  /// Whether detected gestures should provide acoustic and/or haptic feedback.
  ///
  /// For example, on Android a tap will produce a clicking sound and a
  /// long-press will produce a short vibration, when feedback is enabled.
  ///
  /// See also:
  ///
  ///  * [Feedback] for providing platform-specific feedback to certain actions.
  final bool enableFeedback;

  /// Whether to exclude the gestures introduced by this widget from the
  /// semantics tree.
  ///
  /// For example, a long-press gesture for showing a tooltip is usually
  /// excluded because the tooltip itself is included in the semantics
  /// tree directly and so having a gesture to show it would result in
  /// duplication of information.
  final bool excludeFromSemantics;

  /// {@template flutter.material.inkwell.onFocusChange}
  /// Handler called when the focus changes.
  ///
  /// Called with true if this widget's node gains focus, and false if it loses
  /// focus.
  /// {@endtemplate}
  final ValueChanged<bool>? onFocusChange;

  /// {@macro flutter.widgets.Focus.autofocus}
  final bool autofocus;

  /// {@macro flutter.widgets.Focus.focusNode}
  final FocusNode? focusNode;

  /// {@macro flutter.widgets.Focus.canRequestFocus}
  final bool canRequestFocus;

  /// The rectangle to use for the highlight effect and for clipping
  /// the splash effects if [containedInkWell] is true.
  ///
  /// This method is intended to be overridden by descendants that
  /// specialize [InkResponse] for unusual cases. For example,
  /// [TableRowInkWell] implements this method to return the rectangle
  /// corresponding to the row that the widget is in.
  ///
  /// The default behavior returns null, which is equivalent to
  /// returning the referenceBox argument's bounding box (though
  /// slightly more efficient).
  RectCallback? getRectCallback(RenderBox referenceBox) => null;

  /// {@template flutter.material.inkwell.statesController}
  /// Represents the interactive "state" of this widget in terms of
  /// a set of [WidgetState]s, like [WidgetState.pressed] and
  /// [WidgetState.focused].
  ///
  /// Classes based on this one can provide their own
  /// [WidgetStatesController] to which they've added listeners.
  /// They can also update the controller's [WidgetStatesController.value]
  /// however, this may only be done when it's safe to call
  /// [State.setState], like in an event handler.
  /// {@endtemplate}
  final MaterialStatesController? statesController;

  /// The duration of the animation that animates the hover effect.
  ///
  /// The default is 50ms.
  final Duration? hoverDuration;

  @override
  Widget build(BuildContext context) {
    final _ParentInkResponseState? parentState = _ParentInkResponseProvider.maybeOf(context);
    return _InkResponseStateWidget(
      onTap: onTap,
      onTapDown: onTapDown,
      onTapUp: onTapUp,
      onTapCancel: onTapCancel,
      onDoubleTap: onDoubleTap,
      onLongPress: onLongPress,
      onSecondaryTap: onSecondaryTap,
      onSecondaryTapUp: onSecondaryTapUp,
      onSecondaryTapDown: onSecondaryTapDown,
      onSecondaryTapCancel: onSecondaryTapCancel,
      onHighlightChanged: onHighlightChanged,
      onHover: onHover,
      mouseCursor: mouseCursor,
      containedInkWell: containedInkWell,
      highlightShape: highlightShape,
      radius: radius,
      borderRadius: borderRadius,
      customBorder: customBorder,
      focusColor: focusColor,
      hoverColor: hoverColor,
      highlightColor: highlightColor,
      overlayColor: overlayColor,
      splashColor: splashColor,
      splashFactory: splashFactory,
      enableFeedback: enableFeedback,
      excludeFromSemantics: excludeFromSemantics,
      focusNode: focusNode,
      canRequestFocus: canRequestFocus,
      onFocusChange: onFocusChange,
      autofocus: autofocus,
      parentState: parentState,
      getRectCallback: getRectCallback,
      debugCheckContext: debugCheckContext,
      statesController: statesController,
      hoverDuration: hoverDuration,
      child: child,
    );
  }

  /// Asserts that the given context satisfies the prerequisites for
  /// this class.
  ///
  /// This method is intended to be overridden by descendants that
  /// specialize [InkResponse] for unusual cases. For example,
  /// [TableRowInkWell] implements this method to verify that the widget is
  /// in a table.
  @mustCallSuper
  bool debugCheckContext(BuildContext context) {
    assert(debugCheckHasMaterial(context));
    assert(debugCheckHasDirectionality(context));
    return true;
  }
}

class _InkResponseStateWidget extends StatefulWidget {
  const _InkResponseStateWidget({
    this.child,
    this.onTap,
    this.onTapDown,
    this.onTapUp,
    this.onTapCancel,
    this.onDoubleTap,
    this.onLongPress,
    this.onSecondaryTap,
    this.onSecondaryTapUp,
    this.onSecondaryTapDown,
    this.onSecondaryTapCancel,
    this.onHighlightChanged,
    this.onHover,
    this.mouseCursor,
    this.containedInkWell = false,
    this.highlightShape = BoxShape.circle,
    this.radius,
    this.borderRadius,
    this.customBorder,
    this.focusColor,
    this.hoverColor,
    this.highlightColor,
    this.overlayColor,
    this.splashColor,
    this.splashFactory,
    this.enableFeedback = true,
    this.excludeFromSemantics = false,
    this.focusNode,
    this.canRequestFocus = true,
    this.onFocusChange,
    this.autofocus = false,
    this.parentState,
    this.getRectCallback,
    required this.debugCheckContext,
    this.statesController,
    this.hoverDuration,
  });

  final Widget? child;
  final GestureTapCallback? onTap;
  final GestureTapDownCallback? onTapDown;
  final GestureTapUpCallback? onTapUp;
  final GestureTapCallback? onTapCancel;
  final GestureTapCallback? onDoubleTap;
  final GestureLongPressCallback? onLongPress;
  final GestureTapCallback? onSecondaryTap;
  final GestureTapUpCallback? onSecondaryTapUp;
  final GestureTapDownCallback? onSecondaryTapDown;
  final GestureTapCallback? onSecondaryTapCancel;
  final ValueChanged<bool>? onHighlightChanged;
  final ValueChanged<bool>? onHover;
  final MouseCursor? mouseCursor;
  final bool containedInkWell;
  final BoxShape highlightShape;
  final double? radius;
  final BorderRadius? borderRadius;
  final ShapeBorder? customBorder;
  final Color? focusColor;
  final Color? hoverColor;
  final Color? highlightColor;
  final MaterialStateProperty<Color?>? overlayColor;
  final Color? splashColor;
  final InteractiveInkFeatureFactory? splashFactory;
  final bool enableFeedback;
  final bool excludeFromSemantics;
  final ValueChanged<bool>? onFocusChange;
  final bool autofocus;
  final FocusNode? focusNode;
  final bool canRequestFocus;
  final _ParentInkResponseState? parentState;
  final _GetRectCallback? getRectCallback;
  final _CheckContext debugCheckContext;
  final MaterialStatesController? statesController;
  final Duration? hoverDuration;

  @override
  _InkResponseState createState() => _InkResponseState();

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    final List<String> gestures = <String>[
      if (onTap != null) 'tap',
      if (onDoubleTap != null) 'double tap',
      if (onLongPress != null) 'long press',
      if (onTapDown != null) 'tap down',
      if (onTapUp != null) 'tap up',
      if (onTapCancel != null) 'tap cancel',
      if (onSecondaryTap != null) 'secondary tap',
      if (onSecondaryTapUp != null) 'secondary tap up',
      if (onSecondaryTapDown != null) 'secondary tap down',
      if (onSecondaryTapCancel != null) 'secondary tap cancel',
    ];
    properties.add(IterableProperty<String>('gestures', gestures, ifEmpty: '<none>'));
    properties.add(DiagnosticsProperty<MouseCursor>('mouseCursor', mouseCursor));
    properties.add(
      DiagnosticsProperty<bool>('containedInkWell', containedInkWell, level: DiagnosticLevel.fine),
    );
    properties.add(
      DiagnosticsProperty<BoxShape>(
        'highlightShape',
        highlightShape,
        description: '${containedInkWell ? "clipped to " : ""}$highlightShape',
        showName: false,
      ),
    );
  }
}

/// Used to index the allocated highlights for the different types of highlights
/// in [_InkResponseState].
enum _HighlightType { pressed, hover, focus }

class _InkResponseState extends State<_InkResponseStateWidget>
    with AutomaticKeepAliveClientMixin<_InkResponseStateWidget>
    implements _ParentInkResponseState {
  Set<InteractiveInkFeature>? _splashes;
  InteractiveInkFeature? _currentSplash;
  bool _hovering = false;
  final Map<_HighlightType, InkHighlight?> _highlights = <_HighlightType, InkHighlight?>{};
  late final Map<Type, Action<Intent>> _actionMap = <Type, Action<Intent>>{
    ActivateIntent: CallbackAction<ActivateIntent>(onInvoke: activateOnIntent),
    ButtonActivateIntent: CallbackAction<ButtonActivateIntent>(onInvoke: activateOnIntent),
  };
  MaterialStatesController? internalStatesController;

  bool get highlightsExist =>
      _highlights.values.where((InkHighlight? highlight) => highlight != null).isNotEmpty;

  final ObserverList<_ParentInkResponseState> _activeChildren =
      ObserverList<_ParentInkResponseState>();

  static const Duration _activationDuration = Duration(milliseconds: 100);
  Timer? _activationTimer;

  @override
  void markChildInkResponsePressed(_ParentInkResponseState childState, bool value) {
    final bool lastAnyPressed = _anyChildInkResponsePressed;
    if (value) {
      _activeChildren.add(childState);
    } else {
      _activeChildren.remove(childState);
    }
    final bool nowAnyPressed = _anyChildInkResponsePressed;
    if (nowAnyPressed != lastAnyPressed) {
      widget.parentState?.markChildInkResponsePressed(this, nowAnyPressed);
    }
  }

  bool get _anyChildInkResponsePressed => _activeChildren.isNotEmpty;

  void activateOnIntent(Intent? intent) {
    _activationTimer?.cancel();
    _activationTimer = null;
    _startNewSplash(context: context);
    _currentSplash?.confirm();
    _currentSplash = null;
    if (widget.onTap != null) {
      if (widget.enableFeedback) {
        Feedback.forTap(context);
      }
      widget.onTap?.call();
    }
    // Delay the call to `updateHighlight` to simulate a pressed delay
    // and give MaterialStatesController listeners a chance to react.
    _activationTimer = Timer(_activationDuration, () {
      updateHighlight(_HighlightType.pressed, value: false);
    });
  }

  void simulateTap([Intent? intent]) {
    _startNewSplash(context: context);
    handleTap();
  }

  void simulateLongPress() {
    _startNewSplash(context: context);
    handleLongPress();
  }

  void handleStatesControllerChange() {
    // Force a rebuild to resolve widget.overlayColor, widget.mouseCursor
    setState(() {});
  }

  MaterialStatesController get statesController =>
      widget.statesController ?? internalStatesController!;

  void initStatesController() {
    if (widget.statesController == null) {
      internalStatesController = MaterialStatesController();
    }
    statesController.update(MaterialState.disabled, !enabled);
    statesController.addListener(handleStatesControllerChange);
  }

  @override
  void initState() {
    super.initState();
    initStatesController();
    FocusManager.instance.addHighlightModeListener(handleFocusHighlightModeChange);
  }

  @override
  void didUpdateWidget(_InkResponseStateWidget oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (widget.statesController != oldWidget.statesController) {
      oldWidget.statesController?.removeListener(handleStatesControllerChange);
      if (widget.statesController != null) {
        internalStatesController?.dispose();
        internalStatesController = null;
      }
      initStatesController();
    }
    if (widget.radius != oldWidget.radius ||
        widget.highlightShape != oldWidget.highlightShape ||
        widget.borderRadius != oldWidget.borderRadius) {
      final InkHighlight? hoverHighlight = _highlights[_HighlightType.hover];
      if (hoverHighlight != null) {
        hoverHighlight.dispose();
        updateHighlight(_HighlightType.hover, value: _hovering, callOnHover: false);
      }
      final InkHighlight? focusHighlight = _highlights[_HighlightType.focus];
      if (focusHighlight != null) {
        focusHighlight.dispose();
        // Do not call updateFocusHighlights() here because it is called below
      }
    }
    if (widget.customBorder != oldWidget.customBorder) {
      _updateHighlightsAndSplashes();
    }
    if (enabled != isWidgetEnabled(oldWidget)) {
      statesController.update(MaterialState.disabled, !enabled);
      if (!enabled) {
        statesController.update(MaterialState.pressed, false);
        // Remove the existing hover highlight immediately when enabled is false.
        // Do not rely on updateHighlight or InkHighlight.deactivate to not break
        // the expected lifecycle which is updating _hovering when the mouse exit.
        // Manually updating _hovering here or calling InkHighlight.deactivate
        // will lead to onHover not being called or call when it is not allowed.
        final InkHighlight? hoverHighlight = _highlights[_HighlightType.hover];
        hoverHighlight?.dispose();
      }
      // Don't call widget.onHover because many widgets, including the button
      // widgets, apply setState to an ancestor context from onHover.
      updateHighlight(_HighlightType.hover, value: _hovering, callOnHover: false);
    }
    updateFocusHighlights();
  }

  @override
  void dispose() {
    FocusManager.instance.removeHighlightModeListener(handleFocusHighlightModeChange);
    statesController.removeListener(handleStatesControllerChange);
    internalStatesController?.dispose();
    _activationTimer?.cancel();
    _activationTimer = null;
    super.dispose();
  }

  @override
  bool get wantKeepAlive => highlightsExist || (_splashes != null && _splashes!.isNotEmpty);

  Duration getFadeDurationForType(_HighlightType type) {
    switch (type) {
      case _HighlightType.pressed:
        return const Duration(milliseconds: 200);
      case _HighlightType.hover:
      case _HighlightType.focus:
        return widget.hoverDuration ?? const Duration(milliseconds: 50);
    }
  }

  void updateHighlight(_HighlightType type, {required bool value, bool callOnHover = true}) {
    final InkHighlight? highlight = _highlights[type];
    void handleInkRemoval() {
      assert(_highlights[type] != null);
      _highlights[type] = null;
      updateKeepAlive();
    }

    switch (type) {
      case _HighlightType.pressed:
        statesController.update(MaterialState.pressed, value);
      case _HighlightType.hover:
        if (callOnHover) {
          statesController.update(MaterialState.hovered, value);
        }
      case _HighlightType.focus:
        // see handleFocusUpdate()
        break;
    }

    if (type == _HighlightType.pressed) {
      widget.parentState?.markChildInkResponsePressed(this, value);
    }
    if (value == (highlight != null && highlight.active)) {
      return;
    }

    if (value) {
      if (highlight == null) {
        final Color resolvedOverlayColor =
            widget.overlayColor?.resolve(statesController.value) ??
            switch (type) {
              // Use the backwards compatible defaults
              _HighlightType.pressed => widget.highlightColor ?? Theme.of(context).highlightColor,
              _HighlightType.focus => widget.focusColor ?? Theme.of(context).focusColor,
              _HighlightType.hover => widget.hoverColor ?? Theme.of(context).hoverColor,
            };
        final RenderBox referenceBox = context.findRenderObject()! as RenderBox;
        _highlights[type] = InkHighlight(
          controller: Material.of(context),
          referenceBox: referenceBox,
          color: enabled ? resolvedOverlayColor : resolvedOverlayColor.withAlpha(0),
          shape: widget.highlightShape,
          radius: widget.radius,
          borderRadius: widget.borderRadius,
          customBorder: widget.customBorder,
          rectCallback: widget.getRectCallback!(referenceBox),
          onRemoved: handleInkRemoval,
          textDirection: Directionality.of(context),
          fadeDuration: getFadeDurationForType(type),
        );
        updateKeepAlive();
      } else {
        highlight.activate();
      }
    } else {
      highlight!.deactivate();
    }
    assert(value == (_highlights[type] != null && _highlights[type]!.active));

    switch (type) {
      case _HighlightType.pressed:
        widget.onHighlightChanged?.call(value);
      case _HighlightType.hover:
        if (callOnHover) {
          widget.onHover?.call(value);
        }
      case _HighlightType.focus:
        break;
    }
  }

  void _updateHighlightsAndSplashes() {
    for (final InkHighlight? highlight in _highlights.values) {
      highlight?.customBorder = widget.customBorder;
    }
    _currentSplash?.customBorder = widget.customBorder;

    if (_splashes != null && _splashes!.isNotEmpty) {
      for (final InteractiveInkFeature inkFeature in _splashes!) {
        inkFeature.customBorder = widget.customBorder;
      }
    }
  }

  InteractiveInkFeature _createSplash(Offset globalPosition) {
    final MaterialInkController inkController = Material.of(context);
    final RenderBox referenceBox = context.findRenderObject()! as RenderBox;
    final Offset position = referenceBox.globalToLocal(globalPosition);
    final Color color =
        widget.overlayColor?.resolve(statesController.value) ??
        widget.splashColor ??
        Theme.of(context).splashColor;
    final RectCallback? rectCallback =
        widget.containedInkWell ? widget.getRectCallback!(referenceBox) : null;
    final BorderRadius? borderRadius = widget.borderRadius;
    final ShapeBorder? customBorder = widget.customBorder;

    InteractiveInkFeature? splash;
    void onRemoved() {
      if (_splashes != null) {
        assert(_splashes!.contains(splash));
        _splashes!.remove(splash);
        if (_currentSplash == splash) {
          _currentSplash = null;
        }
        updateKeepAlive();
      } // else we're probably in deactivate()
    }

    splash = (widget.splashFactory ?? Theme.of(context).splashFactory).create(
      controller: inkController,
      referenceBox: referenceBox,
      position: position,
      color: color,
      containedInkWell: widget.containedInkWell,
      rectCallback: rectCallback,
      radius: widget.radius,
      borderRadius: borderRadius,
      customBorder: customBorder,
      onRemoved: onRemoved,
      textDirection: Directionality.of(context),
    );

    return splash;
  }

  void handleFocusHighlightModeChange(FocusHighlightMode mode) {
    if (!mounted) {
      return;
    }
    setState(() {
      updateFocusHighlights();
    });
  }

  bool get _shouldShowFocus => switch (MediaQuery.maybeNavigationModeOf(context)) {
    NavigationMode.traditional || null => enabled && _hasFocus,
    NavigationMode.directional => _hasFocus,
  };

  void updateFocusHighlights() {
    final bool showFocus = switch (FocusManager.instance.highlightMode) {
      FocusHighlightMode.touch => false,
      FocusHighlightMode.traditional => _shouldShowFocus,
    };
    updateHighlight(_HighlightType.focus, value: showFocus);
  }

  bool _hasFocus = false;
  void handleFocusUpdate(bool hasFocus) {
    _hasFocus = hasFocus;
    // Set here rather than updateHighlight because this widget's
    // (MaterialState) states include MaterialState.focused if
    // the InkWell _has_ the focus, rather than if it's showing
    // the focus per FocusManager.instance.highlightMode.
    statesController.update(MaterialState.focused, hasFocus);
    updateFocusHighlights();
    widget.onFocusChange?.call(hasFocus);
  }

  void handleAnyTapDown(TapDownDetails details) {
    if (_anyChildInkResponsePressed) {
      return;
    }
    _startNewSplash(details: details);
  }

  void handleTapDown(TapDownDetails details) {
    handleAnyTapDown(details);
    widget.onTapDown?.call(details);
  }

  void handleTapUp(TapUpDetails details) {
    widget.onTapUp?.call(details);
  }

  void handleSecondaryTapDown(TapDownDetails details) {
    handleAnyTapDown(details);
    widget.onSecondaryTapDown?.call(details);
  }

  void handleSecondaryTapUp(TapUpDetails details) {
    widget.onSecondaryTapUp?.call(details);
  }

  void _startNewSplash({TapDownDetails? details, BuildContext? context}) {
    assert(details != null || context != null);

    final Offset globalPosition;
    if (context != null) {
      final RenderBox referenceBox = context.findRenderObject()! as RenderBox;
      assert(
        referenceBox.hasSize,
        'InkResponse must be done with layout before starting a splash.',
      );
      globalPosition = referenceBox.localToGlobal(referenceBox.paintBounds.center);
    } else {
      globalPosition = details!.globalPosition;
    }
    statesController.update(MaterialState.pressed, true); // ... before creating the splash
    final InteractiveInkFeature splash = _createSplash(globalPosition);
    _splashes ??= HashSet<InteractiveInkFeature>();
    _splashes!.add(splash);
    _currentSplash?.cancel();
    _currentSplash = splash;
    updateKeepAlive();
    updateHighlight(_HighlightType.pressed, value: true);
  }

  void handleTap() {
    _currentSplash?.confirm();
    _currentSplash = null;
    updateHighlight(_HighlightType.pressed, value: false);
    if (widget.onTap != null) {
      if (widget.enableFeedback) {
        Feedback.forTap(context);
      }
      widget.onTap?.call();
    }
  }

  void handleTapCancel() {
    _currentSplash?.cancel();
    _currentSplash = null;
    widget.onTapCancel?.call();
    updateHighlight(_HighlightType.pressed, value: false);
  }

  void handleDoubleTap() {
    _currentSplash?.confirm();
    _currentSplash = null;
    updateHighlight(_HighlightType.pressed, value: false);
    widget.onDoubleTap?.call();
  }

  void handleLongPress() {
    _currentSplash?.confirm();
    _currentSplash = null;
    if (widget.onLongPress != null) {
      if (widget.enableFeedback) {
        Feedback.forLongPress(context);
      }
      widget.onLongPress!();
    }
  }

  void handleSecondaryTap() {
    _currentSplash?.confirm();
    _currentSplash = null;
    updateHighlight(_HighlightType.pressed, value: false);
    widget.onSecondaryTap?.call();
  }

  void handleSecondaryTapCancel() {
    _currentSplash?.cancel();
    _currentSplash = null;
    widget.onSecondaryTapCancel?.call();
    updateHighlight(_HighlightType.pressed, value: false);
  }

  @override
  void deactivate() {
    if (_splashes != null) {
      final Set<InteractiveInkFeature> splashes = _splashes!;
      _splashes = null;
      for (final InteractiveInkFeature splash in splashes) {
        splash.dispose();
      }
      _currentSplash = null;
    }
    assert(_currentSplash == null);
    for (final _HighlightType highlight in _highlights.keys) {
      _highlights[highlight]?.dispose();
      _highlights[highlight] = null;
    }
    widget.parentState?.markChildInkResponsePressed(this, false);
    super.deactivate();
  }

  bool isWidgetEnabled(_InkResponseStateWidget widget) {
    return _primaryButtonEnabled(widget) || _secondaryButtonEnabled(widget);
  }

  bool _primaryButtonEnabled(_InkResponseStateWidget widget) {
    return widget.onTap != null ||
        widget.onDoubleTap != null ||
        widget.onLongPress != null ||
        widget.onTapUp != null ||
        widget.onTapDown != null;
  }

  bool _secondaryButtonEnabled(_InkResponseStateWidget widget) {
    return widget.onSecondaryTap != null ||
        widget.onSecondaryTapUp != null ||
        widget.onSecondaryTapDown != null;
  }

  bool get enabled => isWidgetEnabled(widget);
  bool get _primaryEnabled => _primaryButtonEnabled(widget);
  bool get _secondaryEnabled => _secondaryButtonEnabled(widget);

  void handleMouseEnter(PointerEnterEvent event) {
    _hovering = true;
    if (enabled) {
      handleHoverChange();
    }
  }

  void handleMouseExit(PointerExitEvent event) {
    _hovering = false;
    // If the exit occurs after we've been disabled, we still
    // want to take down the highlights and run widget.onHover.
    handleHoverChange();
  }

  void handleHoverChange() {
    updateHighlight(_HighlightType.hover, value: _hovering);
  }

  bool get _canRequestFocus => switch (MediaQuery.maybeNavigationModeOf(context)) {
    NavigationMode.traditional || null => enabled && widget.canRequestFocus,
    NavigationMode.directional => true,
  };

  @override
  Widget build(BuildContext context) {
    assert(widget.debugCheckContext(context));
    super.build(context); // See AutomaticKeepAliveClientMixin.

    final ThemeData theme = Theme.of(context);
    const Set<MaterialState> highlightableStates = <MaterialState>{
      MaterialState.focused,
      MaterialState.hovered,
      MaterialState.pressed,
    };
    final Set<MaterialState> nonHighlightableStates = statesController.value.difference(
      highlightableStates,
    );
    // Each highlightable state will be resolved separately to get the corresponding color.
    // For this resolution to be correct, the non-highlightable states should be preserved.
    final Set<MaterialState> pressed = <MaterialState>{
      ...nonHighlightableStates,
      MaterialState.pressed,
    };
    final Set<MaterialState> focused = <MaterialState>{
      ...nonHighlightableStates,
      MaterialState.focused,
    };
    final Set<MaterialState> hovered = <MaterialState>{
      ...nonHighlightableStates,
      MaterialState.hovered,
    };

    Color getHighlightColorForType(_HighlightType type) {
      return switch (type) {
        // The pressed state triggers a ripple (ink splash), per the current
        // Material Design spec. A separate highlight is no longer used.
        // See https://material.io/design/interaction/states.html#pressed
        _HighlightType.pressed =>
          widget.overlayColor?.resolve(pressed) ?? widget.highlightColor ?? theme.highlightColor,
        _HighlightType.focus =>
          widget.overlayColor?.resolve(focused) ?? widget.focusColor ?? theme.focusColor,
        _HighlightType.hover =>
          widget.overlayColor?.resolve(hovered) ?? widget.hoverColor ?? theme.hoverColor,
      };
    }

    for (final _HighlightType type in _highlights.keys) {
      _highlights[type]?.color = getHighlightColorForType(type);
    }

    _currentSplash?.color =
        widget.overlayColor?.resolve(statesController.value) ??
        widget.splashColor ??
        Theme.of(context).splashColor;

    final MouseCursor effectiveMouseCursor = MaterialStateProperty.resolveAs<MouseCursor>(
      widget.mouseCursor ?? MaterialStateMouseCursor.clickable,
      statesController.value,
    );

    return _ParentInkResponseProvider(
      state: this,
      child: Actions(
        actions: _actionMap,
        child: Focus(
          focusNode: widget.focusNode,
          canRequestFocus: _canRequestFocus,
          onFocusChange: handleFocusUpdate,
          autofocus: widget.autofocus,
          child: MouseRegion(
            cursor: effectiveMouseCursor,
            onEnter: handleMouseEnter,
            onExit: handleMouseExit,
            child: DefaultSelectionStyle.merge(
              mouseCursor: effectiveMouseCursor,
              child: Semantics(
                onTap: widget.excludeFromSemantics || widget.onTap == null ? null : simulateTap,
                onLongPress:
                    widget.excludeFromSemantics || widget.onLongPress == null
                        ? null
                        : simulateLongPress,
                child: GestureDetector(
                  onTapDown: _primaryEnabled ? handleTapDown : null,
                  onTapUp: _primaryEnabled ? handleTapUp : null,
                  onTap: _primaryEnabled ? handleTap : null,
                  onTapCancel: _primaryEnabled ? handleTapCancel : null,
                  onDoubleTap: widget.onDoubleTap != null ? handleDoubleTap : null,
                  onLongPress: widget.onLongPress != null ? handleLongPress : null,
                  onSecondaryTapDown: _secondaryEnabled ? handleSecondaryTapDown : null,
                  onSecondaryTapUp: _secondaryEnabled ? handleSecondaryTapUp : null,
                  onSecondaryTap: _secondaryEnabled ? handleSecondaryTap : null,
                  onSecondaryTapCancel: _secondaryEnabled ? handleSecondaryTapCancel : null,
                  behavior: HitTestBehavior.opaque,
                  excludeFromSemantics: true,
                  child: widget.child,
                ),
              ),
            ),
          ),
        ),
      ),
    );
  }
}

/// A rectangular area of a [Material] that responds to touch.
///
/// For a variant of this widget that does not clip splashes, see [InkResponse].
///
/// The following diagram shows how an [InkWell] looks when tapped, when using
/// default values.
///
/// ![The highlight is a rectangle the size of the box.](https://flutter.github.io/assets-for-api-docs/assets/material/ink_well.png)
///
/// The [InkWell] widget must have a [Material] widget as an ancestor. The
/// [Material] widget is where the ink reactions are actually painted. This
/// matches the Material Design premise wherein the [Material] is what is
/// actually reacting to touches by spreading ink.
///
/// If a Widget uses this class directly, it should include the following line
/// at the top of its build function to call [debugCheckHasMaterial]:
///
/// ```dart
/// assert(debugCheckHasMaterial(context));
/// ```
///
/// ## Troubleshooting
///
/// ### The ink splashes aren't visible!
///
/// If there is an opaque graphic, e.g. painted using a [Container], [Image], or
/// [DecoratedBox], between the [Material] widget and the [InkWell] widget, then
/// the splash won't be visible because it will be under the opaque graphic.
/// This is because ink splashes draw on the underlying [Material] itself, as
/// if the ink was spreading inside the material.
///
/// The [Ink] widget can be used as a replacement for [Image], [Container], or
/// [DecoratedBox] to ensure that the image or decoration also paints in the
/// [Material] itself, below the ink.
///
/// If this is not possible for some reason, e.g. because you are using an
/// opaque [CustomPaint] widget, alternatively consider using a second
/// [Material] above the opaque widget but below the [InkWell] (as an
/// ancestor to the ink well). The [MaterialType.transparency] material
/// kind can be used for this purpose.
///
/// ### InkWell isn't clipping properly
///
/// If you want to clip an InkWell or any [Ink] widgets you need to keep in mind
/// that the [Material] that the Ink will be printed on is responsible for clipping.
/// This means you can't wrap the [Ink] widget in a clipping widget directly,
/// since this will leave the [Material] not clipped (and by extension the printed
/// [Ink] widgets as well).
///
/// An easy solution is to deliberately wrap the [Ink] widgets you want to clip
/// in a [Material], and wrap that in a clipping widget instead. See [Ink] for
/// an example.
///
/// ### The ink splashes don't track the size of an animated container
/// If the size of an InkWell's [Material] ancestor changes while the InkWell's
/// splashes are expanding, you may notice that the splashes aren't clipped
/// correctly. This can't be avoided.
///
/// An example of this situation is as follows:
///
/// {@tool dartpad}
/// Tap the container to cause it to grow. Then, tap it again and hold before
/// the widget reaches its maximum size to observe the clipped ink splash.
///
/// ** See code in examples/api/lib/material/ink_well/ink_well.0.dart **
/// {@end-tool}
///
/// An InkWell's splashes will not properly update to conform to changes if the
/// size of its underlying [Material], where the splashes are rendered, changes
/// during animation. You should avoid using InkWells within [Material] widgets
/// that are changing size.
///
/// See also:
///
///  * [GestureDetector], for listening for gestures without ink splashes.
///  * [ElevatedButton] and [TextButton], two kinds of buttons in Material Design.
///  * [InkResponse], a variant of [InkWell] that doesn't force a rectangular
///    shape on the ink reaction.
class InkWell extends InkResponse {
  /// Creates an ink well.
  ///
  /// Must have an ancestor [Material] widget in which to cause ink reactions.
  const InkWell({
    super.key,
    super.child,
    super.onTap,
    super.onDoubleTap,
    super.onLongPress,
    super.onTapDown,
    super.onTapUp,
    super.onTapCancel,
    super.onSecondaryTap,
    super.onSecondaryTapUp,
    super.onSecondaryTapDown,
    super.onSecondaryTapCancel,
    super.onHighlightChanged,
    super.onHover,
    super.mouseCursor,
    super.focusColor,
    super.hoverColor,
    super.highlightColor,
    super.overlayColor,
    super.splashColor,
    super.splashFactory,
    super.radius,
    super.borderRadius,
    super.customBorder,
    super.enableFeedback,
    super.excludeFromSemantics,
    super.focusNode,
    super.canRequestFocus,
    super.onFocusChange,
    super.autofocus,
    super.statesController,
    super.hoverDuration,
  }) : super(containedInkWell: true, highlightShape: BoxShape.rectangle);
}
