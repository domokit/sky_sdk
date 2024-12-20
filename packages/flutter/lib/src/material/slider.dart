// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'app.dart';
/// @docImport 'checkbox.dart';
/// @docImport 'radio.dart';
/// @docImport 'switch.dart';
/// @docImport 'text_theme.dart';
library;

import 'dart:async';
import 'dart:math' as math;

import 'package:flutter/cupertino.dart';
import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/scheduler.dart' show timeDilation;
import 'package:flutter/services.dart';

import 'color_scheme.dart';
import 'colors.dart';
import 'constants.dart';
import 'debug.dart';
import 'material.dart';
import 'material_state.dart';
import 'slider_theme.dart';
import 'theme.dart';

// Examples can assume:
// int _dollars = 0;
// int _duelCommandment = 1;
// void setState(VoidCallback fn) { }

/// [Slider] uses this callback to paint the value indicator on the overlay.
///
/// Since the value indicator is painted on the Overlay; this method paints the
/// value indicator in a [RenderBox] that appears in the [Overlay].
typedef PaintValueIndicator = void Function(PaintingContext context, Offset offset);

enum _SliderType { material, adaptive }

/// Possible ways for a user to interact with a [Slider].
enum SliderInteraction {
  /// Allows the user to interact with a [Slider] by tapping or sliding anywhere
  /// on the track.
  ///
  /// Essentially all possible interactions are allowed.
  ///
  /// This is different from [SliderInteraction.slideOnly] as when you try
  /// to slide anywhere other than the thumb, the thumb will move to the first
  /// point of contact.
  tapAndSlide,

  /// Allows the user to interact with a [Slider] by only tapping anywhere on
  /// the track.
  ///
  /// Sliding interaction is ignored.
  tapOnly,

  /// Allows the user to interact with a [Slider] only by sliding anywhere on
  /// the track.
  ///
  /// Tapping interaction is ignored.
  slideOnly,

  /// Allows the user to interact with a [Slider] only by sliding the thumb.
  ///
  /// Tapping and sliding interactions on the track are ignored.
  slideThumb,
}

/// A Material Design slider.
///
/// Used to select from a range of values.
///
/// {@youtube 560 315 https://www.youtube.com/watch?v=ufb4gIPDmEs}
///
/// {@tool dartpad}
/// This example showcases non-discrete and discrete [Slider]s.
/// The [Slider]s will show the updated ![Material 3 Design appearance](https://m3.material.io/components/sliders/overview)
/// when setting the [Slider.year2023] flag to false.
///
/// ** See code in examples/api/lib/material/slider/slider.0.dart **
/// {@end-tool}
///
/// {@tool dartpad}
/// This example shows a [Slider] widget using the [Slider.secondaryTrackValue]
/// to show a secondary track in the slider.
///
/// ** See code in examples/api/lib/material/slider/slider.1.dart **
/// {@end-tool}
///
/// A slider can be used to select from either a continuous or a discrete set of
/// values. The default is to use a continuous range of values from [min] to
/// [max]. To use discrete values, use a non-null value for [divisions], which
/// indicates the number of discrete intervals. For example, if [min] is 0.0 and
/// [max] is 50.0 and [divisions] is 5, then the slider can take on the
/// discrete values 0.0, 10.0, 20.0, 30.0, 40.0, and 50.0.
///
/// The terms for the parts of a slider are:
///
///  * The "thumb", which is a shape that slides horizontally when the user
///    drags it.
///  * The "track", which is the line that the slider thumb slides along.
///  * The "value indicator", which is a shape that pops up when the user
///    is dragging the thumb to indicate the value being selected.
///  * The "active" side of the slider is the side between the thumb and the
///    minimum value.
///  * The "inactive" side of the slider is the side between the thumb and the
///    maximum value.
///
/// The slider will be disabled if [onChanged] is null or if the range given by
/// [min]..[max] is empty (i.e. if [min] is equal to [max]).
///
/// The slider widget itself does not maintain any state. Instead, when the state
/// of the slider changes, the widget calls the [onChanged] callback. Most
/// widgets that use a slider will listen for the [onChanged] callback and
/// rebuild the slider with a new [value] to update the visual appearance of the
/// slider. To know when the value starts to change, or when it is done
/// changing, set the optional callbacks [onChangeStart] and/or [onChangeEnd].
///
/// By default, a slider will be as wide as possible, centered vertically. When
/// given unbounded constraints, it will attempt to make the track 144 pixels
/// wide (with margins on each side) and will shrink-wrap vertically.
///
/// Requires one of its ancestors to be a [Material] widget.
///
/// Requires one of its ancestors to be a [MediaQuery] widget. Typically, these
/// are introduced by the [MaterialApp] or [WidgetsApp] widget at the top of
/// your application widget tree.
///
/// To determine how it should be displayed (e.g. colors, thumb shape, etc.),
/// a slider uses the [SliderThemeData] available from either a [SliderTheme]
/// widget or the [ThemeData.sliderTheme] a [Theme] widget above it in the
/// widget tree. You can also override some of the colors with the [activeColor]
/// and [inactiveColor] properties, although more fine-grained control of the
/// look is achieved using a [SliderThemeData].
///
/// See also:
///
///  * [SliderTheme] and [SliderThemeData] for information about controlling
///    the visual appearance of the slider.
///  * [Radio], for selecting among a set of explicit values.
///  * [Checkbox] and [Switch], for toggling a particular value on or off.
///  * <https://material.io/design/components/sliders.html>
///  * [MediaQuery], from which the text scale factor is obtained.
class Slider extends StatefulWidget {
  /// Creates a Material Design slider.
  ///
  /// The slider itself does not maintain any state. Instead, when the state of
  /// the slider changes, the widget calls the [onChanged] callback. Most
  /// widgets that use a slider will listen for the [onChanged] callback and
  /// rebuild the slider with a new [value] to update the visual appearance of
  /// the slider.
  ///
  /// * [value] determines currently selected value for this slider.
  /// * [onChanged] is called while the user is selecting a new value for the
  ///   slider.
  /// * [onChangeStart] is called when the user starts to select a new value for
  ///   the slider.
  /// * [onChangeEnd] is called when the user is done selecting a new value for
  ///   the slider.
  ///
  /// You can override some of the colors with the [activeColor] and
  /// [inactiveColor] properties, although more fine-grained control of the
  /// appearance is achieved using a [SliderThemeData].
  const Slider({
    super.key,
    required this.value,
    this.secondaryTrackValue,
    required this.onChanged,
    this.onChangeStart,
    this.onChangeEnd,
    this.min = 0.0,
    this.max = 1.0,
    this.divisions,
    this.label,
    this.activeColor,
    this.inactiveColor,
    this.secondaryActiveColor,
    this.thumbColor,
    this.overlayColor,
    this.mouseCursor,
    this.semanticFormatterCallback,
    this.focusNode,
    this.autofocus = false,
    this.allowedInteraction,
    this.padding,
    @Deprecated(
      'Use SliderTheme to customize the Slider appearance. '
      'This feature was deprecated after v3.27.0-0.1.pre.',
    )
    this.year2023,
  }) : _sliderType = _SliderType.material,
       assert(min <= max),
       assert(
         value >= min && value <= max,
         'Value $value is not between minimum $min and maximum $max',
       ),
       assert(
         secondaryTrackValue == null || (secondaryTrackValue >= min && secondaryTrackValue <= max),
         'SecondaryValue $secondaryTrackValue is not between $min and $max',
       ),
       assert(divisions == null || divisions > 0);

  /// Creates an adaptive [Slider] based on the target platform, following
  /// Material design's
  /// [Cross-platform guidelines](https://material.io/design/platform-guidance/cross-platform-adaptation.html).
  ///
  /// Creates a [CupertinoSlider] if the target platform is iOS or macOS, creates a
  /// Material Design slider otherwise.
  ///
  /// If a [CupertinoSlider] is created, the following parameters are ignored:
  /// [secondaryTrackValue], [label], [inactiveColor], [secondaryActiveColor],
  /// [semanticFormatterCallback].
  ///
  /// The target platform is based on the current [Theme]: [ThemeData.platform].
  const Slider.adaptive({
    super.key,
    required this.value,
    this.secondaryTrackValue,
    required this.onChanged,
    this.onChangeStart,
    this.onChangeEnd,
    this.min = 0.0,
    this.max = 1.0,
    this.divisions,
    this.label,
    this.mouseCursor,
    this.activeColor,
    this.inactiveColor,
    this.secondaryActiveColor,
    this.thumbColor,
    this.overlayColor,
    this.semanticFormatterCallback,
    this.focusNode,
    this.autofocus = false,
    this.allowedInteraction,
    @Deprecated(
      'Use SliderTheme to customize the Slider appearance. '
      'This feature was deprecated after v3.27.0-0.1.pre.',
    )
    this.year2023,
  }) : _sliderType = _SliderType.adaptive,
       padding = null,
       assert(min <= max),
       assert(
         value >= min && value <= max,
         'Value $value is not between minimum $min and maximum $max',
       ),
       assert(
         secondaryTrackValue == null || (secondaryTrackValue >= min && secondaryTrackValue <= max),
         'SecondaryValue $secondaryTrackValue is not between $min and $max',
       ),
       assert(divisions == null || divisions > 0);

  /// The currently selected value for this slider.
  ///
  /// The slider's thumb is drawn at a position that corresponds to this value.
  final double value;

  /// The secondary track value for this slider.
  ///
  /// If not null, a secondary track using [Slider.secondaryActiveColor] color
  /// is drawn between the thumb and this value, over the inactive track.
  ///
  /// If less than [Slider.value], then the secondary track is not shown.
  ///
  /// It can be ideal for media scenarios such as showing the buffering progress
  /// while the [Slider.value] shows the play progress.
  final double? secondaryTrackValue;

  /// Called during a drag when the user is selecting a new value for the slider
  /// by dragging.
  ///
  /// The slider passes the new value to the callback but does not actually
  /// change state until the parent widget rebuilds the slider with the new
  /// value.
  ///
  /// If null, the slider will be displayed as disabled.
  ///
  /// The callback provided to onChanged should update the state of the parent
  /// [StatefulWidget] using the [State.setState] method, so that the parent
  /// gets rebuilt; for example:
  ///
  /// {@tool snippet}
  ///
  /// ```dart
  /// Slider(
  ///   value: _duelCommandment.toDouble(),
  ///   min: 1.0,
  ///   max: 10.0,
  ///   divisions: 10,
  ///   label: '$_duelCommandment',
  ///   onChanged: (double newValue) {
  ///     setState(() {
  ///       _duelCommandment = newValue.round();
  ///     });
  ///   },
  /// )
  /// ```
  /// {@end-tool}
  ///
  /// See also:
  ///
  ///  * [onChangeStart] for a callback that is called when the user starts
  ///    changing the value.
  ///  * [onChangeEnd] for a callback that is called when the user stops
  ///    changing the value.
  final ValueChanged<double>? onChanged;

  /// Called when the user starts selecting a new value for the slider.
  ///
  /// This callback shouldn't be used to update the slider [value] (use
  /// [onChanged] for that), but rather to be notified when the user has started
  /// selecting a new value by starting a drag or with a tap.
  ///
  /// The value passed will be the last [value] that the slider had before the
  /// change began.
  ///
  /// {@tool snippet}
  ///
  /// ```dart
  /// Slider(
  ///   value: _duelCommandment.toDouble(),
  ///   min: 1.0,
  ///   max: 10.0,
  ///   divisions: 10,
  ///   label: '$_duelCommandment',
  ///   onChanged: (double newValue) {
  ///     setState(() {
  ///       _duelCommandment = newValue.round();
  ///     });
  ///   },
  ///   onChangeStart: (double startValue) {
  ///     print('Started change at $startValue');
  ///   },
  /// )
  /// ```
  /// {@end-tool}
  ///
  /// See also:
  ///
  ///  * [onChangeEnd] for a callback that is called when the value change is
  ///    complete.
  final ValueChanged<double>? onChangeStart;

  /// Called when the user is done selecting a new value for the slider.
  ///
  /// This callback shouldn't be used to update the slider [value] (use
  /// [onChanged] for that), but rather to know when the user has completed
  /// selecting a new [value] by ending a drag or a click.
  ///
  /// {@tool snippet}
  ///
  /// ```dart
  /// Slider(
  ///   value: _duelCommandment.toDouble(),
  ///   min: 1.0,
  ///   max: 10.0,
  ///   divisions: 10,
  ///   label: '$_duelCommandment',
  ///   onChanged: (double newValue) {
  ///     setState(() {
  ///       _duelCommandment = newValue.round();
  ///     });
  ///   },
  ///   onChangeEnd: (double newValue) {
  ///     print('Ended change on $newValue');
  ///   },
  /// )
  /// ```
  /// {@end-tool}
  ///
  /// See also:
  ///
  ///  * [onChangeStart] for a callback that is called when a value change
  ///    begins.
  final ValueChanged<double>? onChangeEnd;

  /// The minimum value the user can select.
  ///
  /// Defaults to 0.0. Must be less than or equal to [max].
  ///
  /// If the [max] is equal to the [min], then the slider is disabled.
  final double min;

  /// The maximum value the user can select.
  ///
  /// Defaults to 1.0. Must be greater than or equal to [min].
  ///
  /// If the [max] is equal to the [min], then the slider is disabled.
  final double max;

  /// The number of discrete divisions.
  ///
  /// Typically used with [label] to show the current discrete value.
  ///
  /// If null, the slider is continuous.
  final int? divisions;

  /// A label to show above the slider when the slider is active and
  /// [SliderThemeData.showValueIndicator] is satisfied.
  ///
  /// It is used to display the value of a discrete slider, and it is displayed
  /// as part of the value indicator shape.
  ///
  /// The label is rendered using the active [ThemeData]'s [TextTheme.bodyLarge]
  /// text style, with the theme data's [ColorScheme.onPrimary] color. The
  /// label's text style can be overridden with
  /// [SliderThemeData.valueIndicatorTextStyle].
  ///
  /// If null, then the value indicator will not be displayed.
  ///
  /// Ignored if this slider is created with [Slider.adaptive].
  ///
  /// See also:
  ///
  ///  * [SliderComponentShape] for how to create a custom value indicator
  ///    shape.
  final String? label;

  /// The color to use for the portion of the slider track that is active.
  ///
  /// The "active" side of the slider is the side between the thumb and the
  /// minimum value.
  ///
  /// If null, [SliderThemeData.activeTrackColor] of the ambient
  /// [SliderTheme] is used. If that is null, [ColorScheme.primary] of the
  /// surrounding [ThemeData] is used.
  ///
  /// Using a [SliderTheme] gives much more fine-grained control over the
  /// appearance of various components of the slider.
  final Color? activeColor;

  /// The color for the inactive portion of the slider track.
  ///
  /// The "inactive" side of the slider is the side between the thumb and the
  /// maximum value.
  ///
  /// If null, [SliderThemeData.inactiveTrackColor] of the ambient [SliderTheme]
  /// is used. If [Slider.year2023] is false and [ThemeData.useMaterial3] is true,
  /// then [ColorScheme.secondaryContainer] is used and if [ThemeData.useMaterial3]
  /// is false, [ColorScheme.primary] with an opacity of 0.24 is used. Otherwise,
  /// [ColorScheme.surfaceContainerHighest] is used.
  ///
  /// Using a [SliderTheme] gives much more fine-grained control over the
  /// appearance of various components of the slider.
  ///
  /// Ignored if this slider is created with [Slider.adaptive].
  final Color? inactiveColor;

  /// The color to use for the portion of the slider track between the thumb and
  /// the [Slider.secondaryTrackValue].
  ///
  /// Defaults to the [SliderThemeData.secondaryActiveTrackColor] of the current
  /// [SliderTheme].
  ///
  /// If that is also null, defaults to [ColorScheme.primary] with an
  /// opacity of 0.54.
  ///
  /// Using a [SliderTheme] gives much more fine-grained control over the
  /// appearance of various components of the slider.
  ///
  /// Ignored if this slider is created with [Slider.adaptive].
  final Color? secondaryActiveColor;

  /// The color of the thumb.
  ///
  /// If this color is null, [Slider] will use [activeColor], If [activeColor]
  /// is also null, [Slider] will use [SliderThemeData.thumbColor].
  ///
  /// If that is also null, defaults to [ColorScheme.primary].
  ///
  /// * [CupertinoSlider] will have a white thumb
  /// (like the native default iOS slider).
  final Color? thumbColor;

  /// The highlight color that's typically used to indicate that
  /// the slider thumb is focused, hovered, or dragged.
  ///
  /// If this property is null, [Slider] will use [activeColor] with
  /// an opacity of 0.12, If null, [SliderThemeData.overlayColor]
  /// will be used.
  ///
  /// If that is also null, If [ThemeData.useMaterial3] is true,
  /// Slider will use [ColorScheme.primary] with an opacity of 0.08 when
  /// slider thumb is hovered and with an opacity of 0.1 when slider thumb
  /// is focused or dragged, If [ThemeData.useMaterial3] is false, defaults
  /// to [ColorScheme.primary] with an opacity of 0.12.
  final MaterialStateProperty<Color?>? overlayColor;

  /// {@template flutter.material.slider.mouseCursor}
  /// The cursor for a mouse pointer when it enters or is hovering over the
  /// widget.
  ///
  /// If [mouseCursor] is a [WidgetStateMouseCursor],
  /// [WidgetStateProperty.resolve] is used for the following [WidgetState]s:
  ///
  ///  * [WidgetState.dragged].
  ///  * [WidgetState.hovered].
  ///  * [WidgetState.focused].
  ///  * [WidgetState.disabled].
  /// {@endtemplate}
  ///
  /// If null, then the value of [SliderThemeData.mouseCursor] is used. If that
  /// is also null, then [WidgetStateMouseCursor.clickable] is used.
  final MouseCursor? mouseCursor;

  /// The callback used to create a semantic value from a slider value.
  ///
  /// Defaults to formatting values as a percentage.
  ///
  /// This is used by accessibility frameworks like TalkBack on Android to
  /// inform users what the currently selected value is with more context.
  ///
  /// {@tool snippet}
  ///
  /// In the example below, a slider for currency values is configured to
  /// announce a value with a currency label.
  ///
  /// ```dart
  /// Slider(
  ///   value: _dollars.toDouble(),
  ///   min: 20.0,
  ///   max: 330.0,
  ///   label: '$_dollars dollars',
  ///   onChanged: (double newValue) {
  ///     setState(() {
  ///       _dollars = newValue.round();
  ///     });
  ///   },
  ///   semanticFormatterCallback: (double newValue) {
  ///     return '${newValue.round()} dollars';
  ///   }
  ///  )
  /// ```
  /// {@end-tool}
  ///
  /// Ignored if this slider is created with [Slider.adaptive]
  final SemanticFormatterCallback? semanticFormatterCallback;

  /// {@macro flutter.widgets.Focus.focusNode}
  final FocusNode? focusNode;

  /// {@macro flutter.widgets.Focus.autofocus}
  final bool autofocus;

  /// Allowed way for the user to interact with the [Slider].
  ///
  /// For example, if this is set to [SliderInteraction.tapOnly], the user can
  /// interact with the slider only by tapping anywhere on the track. Sliding
  /// will have no effect.
  ///
  /// Defaults to [SliderInteraction.tapAndSlide].
  final SliderInteraction? allowedInteraction;

  /// Determines the padding around the [Slider].
  ///
  /// If specified, this padding overrides the default vertical padding of
  /// the [Slider], defaults to the height of the overlay shape, and the
  /// horizontal padding, defaults to the width of the thumb shape or
  /// overlay shape, whichever is larger.
  final EdgeInsetsGeometry? padding;

  /// When true, the [Slider] will use the 2023 Material Design 3 appearance.
  /// Defaults to true.
  ///
  /// If this is set to false, the [Slider] will use the latest Material Design 3
  /// appearance, which was introduced in December 2023.
  ///
  /// If [ThemeData.useMaterial3] is false, then this property is ignored.
  @Deprecated(
    'Use SliderTheme to customize the Slider appearance. '
    'This feature was deprecated after v3.27.0-0.1.pre.',
  )
  final bool? year2023;

  final _SliderType _sliderType;

  @override
  State<Slider> createState() => _SliderState();

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DoubleProperty('value', value));
    properties.add(DoubleProperty('secondaryTrackValue', secondaryTrackValue));
    properties.add(
      ObjectFlagProperty<ValueChanged<double>>('onChanged', onChanged, ifNull: 'disabled'),
    );
    properties.add(ObjectFlagProperty<ValueChanged<double>>.has('onChangeStart', onChangeStart));
    properties.add(ObjectFlagProperty<ValueChanged<double>>.has('onChangeEnd', onChangeEnd));
    properties.add(DoubleProperty('min', min));
    properties.add(DoubleProperty('max', max));
    properties.add(IntProperty('divisions', divisions));
    properties.add(StringProperty('label', label));
    properties.add(ColorProperty('activeColor', activeColor));
    properties.add(ColorProperty('inactiveColor', inactiveColor));
    properties.add(ColorProperty('secondaryActiveColor', secondaryActiveColor));
    properties.add(
      ObjectFlagProperty<ValueChanged<double>>.has(
        'semanticFormatterCallback',
        semanticFormatterCallback,
      ),
    );
    properties.add(ObjectFlagProperty<FocusNode>.has('focusNode', focusNode));
    properties.add(FlagProperty('autofocus', value: autofocus, ifTrue: 'autofocus'));
  }
}

class _SliderState extends State<Slider> with TickerProviderStateMixin {
  static const Duration enableAnimationDuration = Duration(milliseconds: 75);
  static const Duration valueIndicatorAnimationDuration = Duration(milliseconds: 100);

  // Animation controller that is run when the overlay (a.k.a radial reaction)
  // is shown in response to user interaction.
  late AnimationController overlayController;
  // Animation controller that is run when the value indicator is being shown
  // or hidden.
  late AnimationController valueIndicatorController;
  // Animation controller that is run when enabling/disabling the slider.
  late AnimationController enableController;
  // Animation controller that is run when transitioning between one value
  // and the next on a discrete slider.
  late AnimationController positionController;
  Timer? interactionTimer;

  final GlobalKey _renderObjectKey = GlobalKey();

  // Keyboard mapping for a focused slider.
  static const Map<ShortcutActivator, Intent> _traditionalNavShortcutMap =
      <ShortcutActivator, Intent>{
        SingleActivator(LogicalKeyboardKey.arrowUp): _AdjustSliderIntent.up(),
        SingleActivator(LogicalKeyboardKey.arrowDown): _AdjustSliderIntent.down(),
        SingleActivator(LogicalKeyboardKey.arrowLeft): _AdjustSliderIntent.left(),
        SingleActivator(LogicalKeyboardKey.arrowRight): _AdjustSliderIntent.right(),
      };

  // Keyboard mapping for a focused slider when using directional navigation.
  // The vertical inputs are not handled to allow navigating out of the slider.
  static const Map<ShortcutActivator, Intent> _directionalNavShortcutMap =
      <ShortcutActivator, Intent>{
        SingleActivator(LogicalKeyboardKey.arrowLeft): _AdjustSliderIntent.left(),
        SingleActivator(LogicalKeyboardKey.arrowRight): _AdjustSliderIntent.right(),
      };

  // Action mapping for a focused slider.
  late Map<Type, Action<Intent>> _actionMap;

  bool get _enabled => widget.onChanged != null;
  // Value Indicator Animation that appears on the Overlay.
  PaintValueIndicator? paintValueIndicator;

  bool _dragging = false;

  // For discrete sliders, _handleChanged might receive the same value
  // multiple times. To avoid calling widget.onChanged repeatedly, the
  // value from _handleChanged is temporarily saved here.
  double? _currentChangedValue;

  FocusNode? _focusNode;
  FocusNode get focusNode => widget.focusNode ?? _focusNode!;

  @override
  void initState() {
    super.initState();
    overlayController = AnimationController(duration: kRadialReactionDuration, vsync: this);
    valueIndicatorController = AnimationController(
      duration: valueIndicatorAnimationDuration,
      vsync: this,
    );
    enableController = AnimationController(duration: enableAnimationDuration, vsync: this);
    positionController = AnimationController(duration: Duration.zero, vsync: this);
    enableController.value = widget.onChanged != null ? 1.0 : 0.0;
    positionController.value = _convert(widget.value);
    _actionMap = <Type, Action<Intent>>{
      _AdjustSliderIntent: CallbackAction<_AdjustSliderIntent>(onInvoke: _actionHandler),
    };
    if (widget.focusNode == null) {
      // Only create a new node if the widget doesn't have one.
      _focusNode ??= FocusNode();
    }
  }

  @override
  void dispose() {
    interactionTimer?.cancel();
    overlayController.dispose();
    valueIndicatorController.dispose();
    enableController.dispose();
    positionController.dispose();
    overlayEntry?.remove();
    overlayEntry?.dispose();
    overlayEntry = null;
    _focusNode?.dispose();
    super.dispose();
  }

  void _handleChanged(double value) {
    assert(widget.onChanged != null);
    final double lerpValue = _lerp(value);
    if (_currentChangedValue != lerpValue) {
      _currentChangedValue = lerpValue;
      if (_currentChangedValue != widget.value) {
        widget.onChanged!(_currentChangedValue!);
      }
    }
  }

  void _handleDragStart(double value) {
    _dragging = true;
    widget.onChangeStart?.call(_lerp(value));
  }

  void _handleDragEnd(double value) {
    _dragging = false;
    _currentChangedValue = null;
    widget.onChangeEnd?.call(_lerp(value));
  }

  void _actionHandler(_AdjustSliderIntent intent) {
    final TextDirection directionality = Directionality.of(_renderObjectKey.currentContext!);
    final bool shouldIncrease = switch (intent.type) {
      _SliderAdjustmentType.up => true,
      _SliderAdjustmentType.down => false,
      _SliderAdjustmentType.left => directionality == TextDirection.rtl,
      _SliderAdjustmentType.right => directionality == TextDirection.ltr,
    };

    final _RenderSlider slider =
        _renderObjectKey.currentContext!.findRenderObject()! as _RenderSlider;
    return shouldIncrease ? slider.increaseAction() : slider.decreaseAction();
  }

  bool _focused = false;
  void _handleFocusHighlightChanged(bool focused) {
    if (focused != _focused) {
      setState(() {
        _focused = focused;
      });
    }
    showValueIndicator();
  }

  bool _hovering = false;
  void _handleHoverChanged(bool hovering) {
    if (hovering != _hovering) {
      setState(() {
        _hovering = hovering;
      });
    }
  }

  // Returns a number between min and max, proportional to value, which must
  // be between 0.0 and 1.0.
  double _lerp(double value) {
    assert(value >= 0.0);
    assert(value <= 1.0);
    return value * (widget.max - widget.min) + widget.min;
  }

  double _discretize(double value) {
    assert(widget.divisions != null);
    assert(value >= 0.0 && value <= 1.0);

    final int divisions = widget.divisions!;
    return (value * divisions).round() / divisions;
  }

  double _convert(double value) {
    double ret = _unlerp(value);
    if (widget.divisions != null) {
      ret = _discretize(ret);
    }
    return ret;
  }

  // Returns a number between 0.0 and 1.0, given a value between min and max.
  double _unlerp(double value) {
    assert(value <= widget.max);
    assert(value >= widget.min);
    return widget.max > widget.min ? (value - widget.min) / (widget.max - widget.min) : 0.0;
  }

  @override
  Widget build(BuildContext context) {
    assert(debugCheckHasMaterial(context));
    assert(debugCheckHasMediaQuery(context));

    switch (widget._sliderType) {
      case _SliderType.material:
        return _buildMaterialSlider(context);

      case _SliderType.adaptive:
        {
          final ThemeData theme = Theme.of(context);
          switch (theme.platform) {
            case TargetPlatform.android:
            case TargetPlatform.fuchsia:
            case TargetPlatform.linux:
            case TargetPlatform.windows:
              return _buildMaterialSlider(context);
            case TargetPlatform.iOS:
            case TargetPlatform.macOS:
              return _buildCupertinoSlider(context);
          }
        }
    }
  }

  Widget _buildMaterialSlider(BuildContext context) {
    final ThemeData theme = Theme.of(context);
    SliderThemeData sliderTheme = SliderTheme.of(context);
    final bool year2023 = widget.year2023 ?? sliderTheme.year2023 ?? true;
    final SliderThemeData defaults = switch (theme.useMaterial3) {
      true => year2023 ? _SliderDefaultsM3Year2023(context) : _SliderDefaultsM3(context),
      false => _SliderDefaultsM2(context),
    };

    // If the widget has active or inactive colors specified, then we plug them
    // in to the slider theme as best we can. If the developer wants more
    // control than that, then they need to use a SliderTheme. The default
    // colors come from the ThemeData.colorScheme. These colors, along with
    // the default shapes and text styles are aligned to the Material
    // Guidelines.

    const ShowValueIndicator defaultShowValueIndicator = ShowValueIndicator.onlyForDiscrete;
    const SliderInteraction defaultAllowedInteraction = SliderInteraction.tapAndSlide;

    final Set<MaterialState> states = <MaterialState>{
      if (!_enabled) MaterialState.disabled,
      if (_hovering) MaterialState.hovered,
      if (_focused) MaterialState.focused,
      if (_dragging) MaterialState.dragged,
    };

    // The value indicator's color is not the same as the thumb and active track
    // (which can be defined by activeColor) if the
    // RectangularSliderValueIndicatorShape is used. In all other cases, the
    // value indicator is assumed to be the same as the active color.
    final SliderComponentShape valueIndicatorShape =
        sliderTheme.valueIndicatorShape ?? defaults.valueIndicatorShape!;
    final Color valueIndicatorColor;
    if (valueIndicatorShape is RectangularSliderValueIndicatorShape) {
      valueIndicatorColor =
          sliderTheme.valueIndicatorColor ??
          Color.alphaBlend(
            theme.colorScheme.onSurface.withOpacity(0.60),
            theme.colorScheme.surface.withOpacity(0.90),
          );
    } else {
      valueIndicatorColor =
          widget.activeColor ?? sliderTheme.valueIndicatorColor ?? defaults.valueIndicatorColor!;
    }

    Color? effectiveOverlayColor() {
      return widget.overlayColor?.resolve(states) ??
          widget.activeColor?.withOpacity(0.12) ??
          MaterialStateProperty.resolveAs<Color?>(sliderTheme.overlayColor, states) ??
          MaterialStateProperty.resolveAs<Color?>(defaults.overlayColor, states);
    }

    TextStyle valueIndicatorTextStyle =
        sliderTheme.valueIndicatorTextStyle ?? defaults.valueIndicatorTextStyle!;
    if (MediaQuery.boldTextOf(context)) {
      valueIndicatorTextStyle = valueIndicatorTextStyle.merge(
        const TextStyle(fontWeight: FontWeight.bold),
      );
    }

    sliderTheme = sliderTheme.copyWith(
      trackHeight: sliderTheme.trackHeight ?? defaults.trackHeight,
      activeTrackColor:
          widget.activeColor ?? sliderTheme.activeTrackColor ?? defaults.activeTrackColor,
      inactiveTrackColor:
          widget.inactiveColor ?? sliderTheme.inactiveTrackColor ?? defaults.inactiveTrackColor,
      secondaryActiveTrackColor:
          widget.secondaryActiveColor ??
          sliderTheme.secondaryActiveTrackColor ??
          defaults.secondaryActiveTrackColor,
      disabledActiveTrackColor:
          sliderTheme.disabledActiveTrackColor ?? defaults.disabledActiveTrackColor,
      disabledInactiveTrackColor:
          sliderTheme.disabledInactiveTrackColor ?? defaults.disabledInactiveTrackColor,
      disabledSecondaryActiveTrackColor:
          sliderTheme.disabledSecondaryActiveTrackColor ??
          defaults.disabledSecondaryActiveTrackColor,
      activeTickMarkColor:
          widget.inactiveColor ?? sliderTheme.activeTickMarkColor ?? defaults.activeTickMarkColor,
      inactiveTickMarkColor:
          widget.activeColor ?? sliderTheme.inactiveTickMarkColor ?? defaults.inactiveTickMarkColor,
      disabledActiveTickMarkColor:
          sliderTheme.disabledActiveTickMarkColor ?? defaults.disabledActiveTickMarkColor,
      disabledInactiveTickMarkColor:
          sliderTheme.disabledInactiveTickMarkColor ?? defaults.disabledInactiveTickMarkColor,
      thumbColor:
          widget.thumbColor ?? widget.activeColor ?? sliderTheme.thumbColor ?? defaults.thumbColor,
      disabledThumbColor: sliderTheme.disabledThumbColor ?? defaults.disabledThumbColor,
      overlayColor: effectiveOverlayColor(),
      valueIndicatorColor: valueIndicatorColor,
      trackShape: sliderTheme.trackShape ?? defaults.trackShape,
      tickMarkShape: sliderTheme.tickMarkShape ?? defaults.tickMarkShape,
      thumbShape: sliderTheme.thumbShape ?? defaults.thumbShape,
      overlayShape: sliderTheme.overlayShape ?? defaults.overlayShape,
      valueIndicatorShape: valueIndicatorShape,
      showValueIndicator: sliderTheme.showValueIndicator ?? defaultShowValueIndicator,
      valueIndicatorTextStyle: valueIndicatorTextStyle,
      padding: widget.padding ?? sliderTheme.padding,
      thumbSize: sliderTheme.thumbSize ?? defaults.thumbSize,
      trackGap: sliderTheme.trackGap ?? defaults.trackGap,
    );
    final MouseCursor effectiveMouseCursor =
        MaterialStateProperty.resolveAs<MouseCursor?>(widget.mouseCursor, states) ??
        sliderTheme.mouseCursor?.resolve(states) ??
        MaterialStateMouseCursor.clickable.resolve(states);
    final SliderInteraction effectiveAllowedInteraction =
        widget.allowedInteraction ?? sliderTheme.allowedInteraction ?? defaultAllowedInteraction;

    // This size is used as the max bounds for the painting of the value
    // indicators It must be kept in sync with the function with the same name
    // in range_slider.dart.
    Size screenSize() => MediaQuery.sizeOf(context);

    VoidCallback? handleDidGainAccessibilityFocus;
    switch (theme.platform) {
      case TargetPlatform.android:
      case TargetPlatform.fuchsia:
      case TargetPlatform.iOS:
      case TargetPlatform.linux:
      case TargetPlatform.macOS:
        break;
      case TargetPlatform.windows:
        handleDidGainAccessibilityFocus = () {
          // Automatically activate the slider when it receives a11y focus.
          if (!focusNode.hasFocus && focusNode.canRequestFocus) {
            focusNode.requestFocus();
          }
        };
    }

    final Map<ShortcutActivator, Intent> shortcutMap = switch (MediaQuery.navigationModeOf(
      context,
    )) {
      NavigationMode.directional => _directionalNavShortcutMap,
      NavigationMode.traditional => _traditionalNavShortcutMap,
    };

    final double fontSize = sliderTheme.valueIndicatorTextStyle?.fontSize ?? kDefaultFontSize;
    final double fontSizeToScale = fontSize == 0.0 ? kDefaultFontSize : fontSize;
    final TextScaler textScaler =
        theme.useMaterial3
            // TODO(tahatesser): This is an eye-balled value.
            // This needs to be updated when accessibility
            // guidelines are available on the material specs page
            // https://m3.material.io/components/sliders/accessibility.
            ? MediaQuery.textScalerOf(context).clamp(maxScaleFactor: 1.3)
            : MediaQuery.textScalerOf(context);
    final double effectiveTextScale = textScaler.scale(fontSizeToScale) / fontSizeToScale;

    Widget result = CompositedTransformTarget(
      link: _layerLink,
      child: _SliderRenderObjectWidget(
        key: _renderObjectKey,
        value: _convert(widget.value),
        secondaryTrackValue:
            (widget.secondaryTrackValue != null) ? _convert(widget.secondaryTrackValue!) : null,
        divisions: widget.divisions,
        label: widget.label,
        sliderTheme: sliderTheme,
        textScaleFactor: effectiveTextScale,
        screenSize: screenSize(),
        onChanged: (widget.onChanged != null) && (widget.max > widget.min) ? _handleChanged : null,
        onChangeStart: _handleDragStart,
        onChangeEnd: _handleDragEnd,
        state: this,
        semanticFormatterCallback: widget.semanticFormatterCallback,
        hasFocus: _focused,
        hovering: _hovering,
        allowedInteraction: effectiveAllowedInteraction,
      ),
    );

    final EdgeInsetsGeometry? padding = widget.padding ?? sliderTheme.padding;
    if (padding != null) {
      result = Padding(padding: padding, child: result);
    }

    return Semantics(
      container: true,
      slider: true,
      onDidGainAccessibilityFocus: handleDidGainAccessibilityFocus,
      child: FocusableActionDetector(
        actions: _actionMap,
        shortcuts: shortcutMap,
        focusNode: focusNode,
        autofocus: widget.autofocus,
        enabled: _enabled,
        onShowFocusHighlight: _handleFocusHighlightChanged,
        onShowHoverHighlight: _handleHoverChanged,
        mouseCursor: effectiveMouseCursor,
        child: result,
      ),
    );
  }

  Widget _buildCupertinoSlider(BuildContext context) {
    // The render box of a slider has a fixed height but takes up the available
    // width. Wrapping the [CupertinoSlider] in this manner will help maintain
    // the same size.
    return SizedBox(
      width: double.infinity,
      child: CupertinoSlider(
        value: widget.value,
        onChanged: widget.onChanged,
        onChangeStart: widget.onChangeStart,
        onChangeEnd: widget.onChangeEnd,
        min: widget.min,
        max: widget.max,
        divisions: widget.divisions,
        activeColor: widget.activeColor,
        thumbColor: widget.thumbColor ?? CupertinoColors.white,
      ),
    );
  }

  final LayerLink _layerLink = LayerLink();

  OverlayEntry? overlayEntry;

  void showValueIndicator() {
    if (overlayEntry == null) {
      overlayEntry = OverlayEntry(
        builder: (BuildContext context) {
          return CompositedTransformFollower(
            link: _layerLink,
            child: _ValueIndicatorRenderObjectWidget(state: this),
          );
        },
      );
      Overlay.of(context, debugRequiredFor: widget).insert(overlayEntry!);
    }
  }
}

class _SliderRenderObjectWidget extends LeafRenderObjectWidget {
  const _SliderRenderObjectWidget({
    super.key,
    required this.value,
    required this.secondaryTrackValue,
    required this.divisions,
    required this.label,
    required this.sliderTheme,
    required this.textScaleFactor,
    required this.screenSize,
    required this.onChanged,
    required this.onChangeStart,
    required this.onChangeEnd,
    required this.state,
    required this.semanticFormatterCallback,
    required this.hasFocus,
    required this.hovering,
    required this.allowedInteraction,
  });

  final double value;
  final double? secondaryTrackValue;
  final int? divisions;
  final String? label;
  final SliderThemeData sliderTheme;
  final double textScaleFactor;
  final Size screenSize;
  final ValueChanged<double>? onChanged;
  final ValueChanged<double>? onChangeStart;
  final ValueChanged<double>? onChangeEnd;
  final SemanticFormatterCallback? semanticFormatterCallback;
  final _SliderState state;
  final bool hasFocus;
  final bool hovering;
  final SliderInteraction allowedInteraction;

  @override
  _RenderSlider createRenderObject(BuildContext context) {
    return _RenderSlider(
      value: value,
      secondaryTrackValue: secondaryTrackValue,
      divisions: divisions,
      label: label,
      sliderTheme: sliderTheme,
      textScaleFactor: textScaleFactor,
      screenSize: screenSize,
      onChanged: onChanged,
      onChangeStart: onChangeStart,
      onChangeEnd: onChangeEnd,
      state: state,
      textDirection: Directionality.of(context),
      semanticFormatterCallback: semanticFormatterCallback,
      platform: Theme.of(context).platform,
      hasFocus: hasFocus,
      hovering: hovering,
      gestureSettings: MediaQuery.gestureSettingsOf(context),
      allowedInteraction: allowedInteraction,
    );
  }

  @override
  void updateRenderObject(BuildContext context, _RenderSlider renderObject) {
    renderObject
      // We should update the `divisions` ahead of `value`, because the `value`
      // setter dependent on the `divisions`.
      ..divisions = divisions
      ..value = value
      ..secondaryTrackValue = secondaryTrackValue
      ..label = label
      ..sliderTheme = sliderTheme
      ..textScaleFactor = textScaleFactor
      ..screenSize = screenSize
      ..onChanged = onChanged
      ..onChangeStart = onChangeStart
      ..onChangeEnd = onChangeEnd
      ..textDirection = Directionality.of(context)
      ..semanticFormatterCallback = semanticFormatterCallback
      ..platform = Theme.of(context).platform
      ..hasFocus = hasFocus
      ..hovering = hovering
      ..gestureSettings = MediaQuery.gestureSettingsOf(context)
      ..allowedInteraction = allowedInteraction;
    // Ticker provider cannot change since there's a 1:1 relationship between
    // the _SliderRenderObjectWidget object and the _SliderState object.
  }
}

class _RenderSlider extends RenderBox with RelayoutWhenSystemFontsChangeMixin {
  _RenderSlider({
    required double value,
    required double? secondaryTrackValue,
    required int? divisions,
    required String? label,
    required SliderThemeData sliderTheme,
    required double textScaleFactor,
    required Size screenSize,
    required TargetPlatform platform,
    required ValueChanged<double>? onChanged,
    required SemanticFormatterCallback? semanticFormatterCallback,
    required this.onChangeStart,
    required this.onChangeEnd,
    required _SliderState state,
    required TextDirection textDirection,
    required bool hasFocus,
    required bool hovering,
    required DeviceGestureSettings gestureSettings,
    required SliderInteraction allowedInteraction,
  }) : assert(value >= 0.0 && value <= 1.0),
       assert(
         secondaryTrackValue == null || (secondaryTrackValue >= 0.0 && secondaryTrackValue <= 1.0),
       ),
       _platform = platform,
       _semanticFormatterCallback = semanticFormatterCallback,
       _label = label,
       _value = value,
       _secondaryTrackValue = secondaryTrackValue,
       _divisions = divisions,
       _sliderTheme = sliderTheme,
       _textScaleFactor = textScaleFactor,
       _screenSize = screenSize,
       _onChanged = onChanged,
       _state = state,
       _textDirection = textDirection,
       _hasFocus = hasFocus,
       _hovering = hovering,
       _allowedInteraction = allowedInteraction {
    _updateLabelPainter();
    final GestureArenaTeam team = GestureArenaTeam();
    _drag =
        HorizontalDragGestureRecognizer()
          ..team = team
          ..onStart = _handleDragStart
          ..onUpdate = _handleDragUpdate
          ..onEnd = _handleDragEnd
          ..onCancel = _endInteraction
          ..gestureSettings = gestureSettings;
    _tap =
        TapGestureRecognizer()
          ..team = team
          ..onTapDown = _handleTapDown
          ..onTapUp = _handleTapUp
          ..gestureSettings = gestureSettings;
    _overlayAnimation = CurvedAnimation(
      parent: _state.overlayController,
      curve: Curves.fastOutSlowIn,
    );
    _valueIndicatorAnimation = CurvedAnimation(
      parent: _state.valueIndicatorController,
      curve: Curves.fastOutSlowIn,
    )..addStatusListener((AnimationStatus status) {
      if (status.isDismissed) {
        _state.overlayEntry?.remove();
        _state.overlayEntry?.dispose();
        _state.overlayEntry = null;
      }
    });
    _enableAnimation = CurvedAnimation(parent: _state.enableController, curve: Curves.easeInOut);
  }
  static const Duration _positionAnimationDuration = Duration(milliseconds: 75);
  static const Duration _minimumInteractionTime = Duration(milliseconds: 500);

  // This value is the touch target, 48, multiplied by 3.
  static const double _minPreferredTrackWidth = 144.0;

  // Compute the largest width and height needed to paint the slider shapes,
  // other than the track shape. It is assumed that these shapes are vertically
  // centered on the track.
  double get _maxSliderPartWidth =>
      _sliderPartSizes.map((Size size) => size.width).reduce(math.max);
  double get _maxSliderPartHeight =>
      _sliderPartSizes.map((Size size) => size.height).reduce(math.max);
  double get _thumbSizeHeight =>
      _sliderTheme.thumbShape!.getPreferredSize(isInteractive, isDiscrete).height;
  double get _overlayHeight =>
      _sliderTheme.overlayShape!.getPreferredSize(isInteractive, isDiscrete).height;
  List<Size> get _sliderPartSizes => <Size>[
    Size(
      _sliderTheme.overlayShape!.getPreferredSize(isInteractive, isDiscrete).width,
      _sliderTheme.padding != null ? _thumbSizeHeight : _overlayHeight,
    ),
    _sliderTheme.thumbShape!.getPreferredSize(isInteractive, isDiscrete),
    _sliderTheme.tickMarkShape!.getPreferredSize(
      isEnabled: isInteractive,
      sliderTheme: sliderTheme,
    ),
  ];
  double get _minPreferredTrackHeight => _sliderTheme.trackHeight!;

  final _SliderState _state;
  late CurvedAnimation _overlayAnimation;
  late CurvedAnimation _valueIndicatorAnimation;
  late CurvedAnimation _enableAnimation;
  final TextPainter _labelPainter = TextPainter();
  late HorizontalDragGestureRecognizer _drag;
  late TapGestureRecognizer _tap;
  bool _active = false;
  double _currentDragValue = 0.0;
  Rect? overlayRect;

  // This rect is used in gesture calculations, where the gesture coordinates
  // are relative to the sliders origin. Therefore, the offset is passed as
  // (0,0).
  Rect get _trackRect => _sliderTheme.trackShape!.getPreferredRect(
    parentBox: this,
    sliderTheme: _sliderTheme,
    isDiscrete: false,
  );

  bool get isInteractive => onChanged != null;

  bool get isDiscrete => divisions != null && divisions! > 0;

  double get value => _value;
  double _value;
  set value(double newValue) {
    assert(newValue >= 0.0 && newValue <= 1.0);
    final double convertedValue = isDiscrete ? _discretize(newValue) : newValue;
    if (convertedValue == _value) {
      return;
    }
    _value = convertedValue;
    if (isDiscrete) {
      // Reset the duration to match the distance that we're traveling, so that
      // whatever the distance, we still do it in _positionAnimationDuration,
      // and if we get re-targeted in the middle, it still takes that long to
      // get to the new location.
      final double distance = (_value - _state.positionController.value).abs();
      _state.positionController.duration =
          distance != 0.0 ? _positionAnimationDuration * (1.0 / distance) : Duration.zero;
      _state.positionController.animateTo(convertedValue, curve: Curves.easeInOut);
    } else {
      _state.positionController.value = convertedValue;
    }
    markNeedsSemanticsUpdate();
  }

  double? get secondaryTrackValue => _secondaryTrackValue;
  double? _secondaryTrackValue;
  set secondaryTrackValue(double? newValue) {
    assert(newValue == null || (newValue >= 0.0 && newValue <= 1.0));
    if (newValue == _secondaryTrackValue) {
      return;
    }
    _secondaryTrackValue = newValue;
    markNeedsSemanticsUpdate();
  }

  DeviceGestureSettings? get gestureSettings => _drag.gestureSettings;
  set gestureSettings(DeviceGestureSettings? gestureSettings) {
    _drag.gestureSettings = gestureSettings;
    _tap.gestureSettings = gestureSettings;
  }

  TargetPlatform _platform;
  TargetPlatform get platform => _platform;
  set platform(TargetPlatform value) {
    if (_platform == value) {
      return;
    }
    _platform = value;
    markNeedsSemanticsUpdate();
  }

  SemanticFormatterCallback? _semanticFormatterCallback;
  SemanticFormatterCallback? get semanticFormatterCallback => _semanticFormatterCallback;
  set semanticFormatterCallback(SemanticFormatterCallback? value) {
    if (_semanticFormatterCallback == value) {
      return;
    }
    _semanticFormatterCallback = value;
    markNeedsSemanticsUpdate();
  }

  int? get divisions => _divisions;
  int? _divisions;
  set divisions(int? value) {
    if (value == _divisions) {
      return;
    }
    _divisions = value;
    markNeedsPaint();
  }

  String? get label => _label;
  String? _label;
  set label(String? value) {
    if (value == _label) {
      return;
    }
    _label = value;
    _updateLabelPainter();
  }

  SliderThemeData get sliderTheme => _sliderTheme;
  SliderThemeData _sliderTheme;
  set sliderTheme(SliderThemeData value) {
    if (value == _sliderTheme) {
      return;
    }
    _sliderTheme = value;
    _updateLabelPainter();
  }

  double get textScaleFactor => _textScaleFactor;
  double _textScaleFactor;
  set textScaleFactor(double value) {
    if (value == _textScaleFactor) {
      return;
    }
    _textScaleFactor = value;
    _updateLabelPainter();
  }

  Size get screenSize => _screenSize;
  Size _screenSize;
  set screenSize(Size value) {
    if (value == _screenSize) {
      return;
    }
    _screenSize = value;
    markNeedsPaint();
  }

  ValueChanged<double>? get onChanged => _onChanged;
  ValueChanged<double>? _onChanged;
  set onChanged(ValueChanged<double>? value) {
    if (value == _onChanged) {
      return;
    }
    final bool wasInteractive = isInteractive;
    _onChanged = value;
    if (wasInteractive != isInteractive) {
      if (isInteractive) {
        _state.enableController.forward();
      } else {
        _state.enableController.reverse();
      }
      markNeedsPaint();
      markNeedsSemanticsUpdate();
    }
  }

  ValueChanged<double>? onChangeStart;
  ValueChanged<double>? onChangeEnd;

  TextDirection get textDirection => _textDirection;
  TextDirection _textDirection;
  set textDirection(TextDirection value) {
    if (value == _textDirection) {
      return;
    }
    _textDirection = value;
    _updateLabelPainter();
  }

  /// True if this slider has the input focus.
  bool get hasFocus => _hasFocus;
  bool _hasFocus;
  set hasFocus(bool value) {
    if (value == _hasFocus) {
      return;
    }
    _hasFocus = value;
    _updateForFocus(_hasFocus);
    markNeedsSemanticsUpdate();
  }

  /// True if this slider is being hovered over by a pointer.
  bool get hovering => _hovering;
  bool _hovering;
  set hovering(bool value) {
    if (value == _hovering) {
      return;
    }
    _hovering = value;
    _updateForHover(_hovering);
  }

  /// True if the slider is interactive and the slider thumb is being
  /// hovered over by a pointer.
  bool _hoveringThumb = false;
  bool get hoveringThumb => _hoveringThumb;
  set hoveringThumb(bool value) {
    if (value == _hoveringThumb) {
      return;
    }
    _hoveringThumb = value;
    _updateForHover(_hovering);
  }

  SliderInteraction _allowedInteraction;
  SliderInteraction get allowedInteraction => _allowedInteraction;
  set allowedInteraction(SliderInteraction value) {
    if (value == _allowedInteraction) {
      return;
    }
    _allowedInteraction = value;
    markNeedsSemanticsUpdate();
  }

  void _updateForFocus(bool focused) {
    if (focused) {
      _state.overlayController.forward();
      if (showValueIndicator) {
        _state.valueIndicatorController.forward();
      }
    } else {
      _state.overlayController.reverse();
      if (showValueIndicator) {
        _state.valueIndicatorController.reverse();
      }
    }
  }

  void _updateForHover(bool hovered) {
    // Only show overlay when pointer is hovering the thumb.
    if (hovered && hoveringThumb) {
      _state.overlayController.forward();
    } else {
      // Only remove overlay when Slider is inactive and unfocused.
      if (!_active && !hasFocus) {
        _state.overlayController.reverse();
      }
    }
  }

  bool get showValueIndicator => switch (_sliderTheme.showValueIndicator!) {
    ShowValueIndicator.onlyForDiscrete => isDiscrete,
    ShowValueIndicator.onlyForContinuous => !isDiscrete,
    ShowValueIndicator.always => true,
    ShowValueIndicator.never => false,
  };

  double get _adjustmentUnit {
    switch (_platform) {
      case TargetPlatform.iOS:
      case TargetPlatform.macOS:
        // Matches iOS implementation of material slider.
        return 0.1;
      case TargetPlatform.android:
      case TargetPlatform.fuchsia:
      case TargetPlatform.linux:
      case TargetPlatform.windows:
        // Matches Android implementation of material slider.
        return 0.05;
    }
  }

  void _updateLabelPainter() {
    if (label != null) {
      _labelPainter
        ..text = TextSpan(style: _sliderTheme.valueIndicatorTextStyle, text: label)
        ..textDirection = textDirection
        ..textScaleFactor = textScaleFactor
        ..layout();
    } else {
      _labelPainter.text = null;
    }
    // Changing the textDirection can result in the layout changing, because the
    // bidi algorithm might line up the glyphs differently which can result in
    // different ligatures, different shapes, etc. So we always markNeedsLayout.
    markNeedsLayout();
  }

  @override
  void systemFontsDidChange() {
    super.systemFontsDidChange();
    _labelPainter.markNeedsLayout();
    _updateLabelPainter();
  }

  @override
  void attach(PipelineOwner owner) {
    super.attach(owner);
    _overlayAnimation.addListener(markNeedsPaint);
    _valueIndicatorAnimation.addListener(markNeedsPaint);
    _enableAnimation.addListener(markNeedsPaint);
    _state.positionController.addListener(markNeedsPaint);
  }

  @override
  void detach() {
    _overlayAnimation.removeListener(markNeedsPaint);
    _valueIndicatorAnimation.removeListener(markNeedsPaint);
    _enableAnimation.removeListener(markNeedsPaint);
    _state.positionController.removeListener(markNeedsPaint);
    super.detach();
  }

  @override
  void dispose() {
    _drag.dispose();
    _tap.dispose();
    _labelPainter.dispose();
    _enableAnimation.dispose();
    _valueIndicatorAnimation.dispose();
    _overlayAnimation.dispose();
    super.dispose();
  }

  double _getValueFromVisualPosition(double visualPosition) {
    return switch (textDirection) {
      TextDirection.rtl => 1.0 - visualPosition,
      TextDirection.ltr => visualPosition,
    };
  }

  double _getValueFromGlobalPosition(Offset globalPosition) {
    final double visualPosition =
        (globalToLocal(globalPosition).dx - _trackRect.left) / _trackRect.width;
    return _getValueFromVisualPosition(visualPosition);
  }

  double _discretize(double value) {
    double result = clampDouble(value, 0.0, 1.0);
    if (isDiscrete) {
      result = (result * divisions!).round() / divisions!;
    }
    return result;
  }

  void _startInteraction(Offset globalPosition) {
    if (!_state.mounted) {
      return;
    }
    _state.showValueIndicator();
    if (!_active && isInteractive) {
      switch (allowedInteraction) {
        case SliderInteraction.tapAndSlide:
        case SliderInteraction.tapOnly:
          _active = true;
          _currentDragValue = _getValueFromGlobalPosition(globalPosition);
        case SliderInteraction.slideThumb:
          if (_isPointerOnOverlay(globalPosition)) {
            _active = true;
            _currentDragValue = value;
          }
        case SliderInteraction.slideOnly:
          onChangeStart?.call(_discretize(value));
      }

      if (_active) {
        // We supply the *current* value as the start location, so that if we have
        // a tap, it consists of a call to onChangeStart with the previous value and
        // a call to onChangeEnd with the new value.
        onChangeStart?.call(_discretize(value));
        onChanged!(_discretize(_currentDragValue));
        _state.overlayController.forward();
        if (showValueIndicator) {
          _state.valueIndicatorController.forward();
          _state.interactionTimer?.cancel();
          _state.interactionTimer = Timer(_minimumInteractionTime * timeDilation, () {
            _state.interactionTimer = null;
            if (!_active && _state.valueIndicatorController.isCompleted) {
              _state.valueIndicatorController.reverse();
            }
          });
        }
      }
    }
  }

  void _endInteraction() {
    if (!_state.mounted) {
      return;
    }

    if (_active && _state.mounted) {
      onChangeEnd?.call(_discretize(_currentDragValue));
      _active = false;
      _currentDragValue = 0.0;
      _state.overlayController.reverse();
      if (showValueIndicator && _state.interactionTimer == null) {
        _state.valueIndicatorController.reverse();
      }
    }
  }

  void _handleDragStart(DragStartDetails details) {
    _startInteraction(details.globalPosition);
  }

  void _handleDragUpdate(DragUpdateDetails details) {
    if (!_state.mounted) {
      return;
    }

    // for slide only, there is no start interaction trigger, so _active
    // will be false and needs to be made true.
    if (!_active && allowedInteraction == SliderInteraction.slideOnly) {
      _active = true;
      _currentDragValue = value;
    }

    switch (allowedInteraction) {
      case SliderInteraction.tapAndSlide:
      case SliderInteraction.slideOnly:
      case SliderInteraction.slideThumb:
        if (_active && isInteractive) {
          final double valueDelta = details.primaryDelta! / _trackRect.width;
          _currentDragValue += switch (textDirection) {
            TextDirection.rtl => -valueDelta,
            TextDirection.ltr => valueDelta,
          };
          onChanged!(_discretize(_currentDragValue));
        }
      case SliderInteraction.tapOnly:
        // cannot slide (drag) as its tapOnly.
        break;
    }
  }

  void _handleDragEnd(DragEndDetails details) {
    _endInteraction();
  }

  void _handleTapDown(TapDownDetails details) {
    _startInteraction(details.globalPosition);
  }

  void _handleTapUp(TapUpDetails details) {
    _endInteraction();
  }

  bool _isPointerOnOverlay(Offset globalPosition) {
    return overlayRect!.contains(globalToLocal(globalPosition));
  }

  @override
  bool hitTestSelf(Offset position) => true;

  @override
  void handleEvent(PointerEvent event, BoxHitTestEntry entry) {
    if (!_state.mounted) {
      return;
    }
    assert(debugHandleEvent(event, entry));
    if (event is PointerDownEvent && isInteractive) {
      // We need to add the drag first so that it has priority.
      _drag.addPointer(event);
      _tap.addPointer(event);
    }
    if (isInteractive && overlayRect != null) {
      hoveringThumb = overlayRect!.contains(event.localPosition);
    }
  }

  @override
  double computeMinIntrinsicWidth(double height) => _minPreferredTrackWidth + _maxSliderPartWidth;

  @override
  double computeMaxIntrinsicWidth(double height) => _minPreferredTrackWidth + _maxSliderPartWidth;

  @override
  double computeMinIntrinsicHeight(double width) =>
      math.max(_minPreferredTrackHeight, _maxSliderPartHeight);

  @override
  double computeMaxIntrinsicHeight(double width) =>
      math.max(_minPreferredTrackHeight, _maxSliderPartHeight);

  @override
  bool get sizedByParent => true;

  @override
  Size computeDryLayout(BoxConstraints constraints) {
    return Size(
      constraints.hasBoundedWidth
          ? constraints.maxWidth
          : _minPreferredTrackWidth + _maxSliderPartWidth,
      constraints.hasBoundedHeight
          ? constraints.maxHeight
          : math.max(_minPreferredTrackHeight, _maxSliderPartHeight),
    );
  }

  @override
  void paint(PaintingContext context, Offset offset) {
    final double controllerValue = _state.positionController.value;

    // The visual position is the position of the thumb from 0 to 1 from left
    // to right. In left to right, this is the same as the value, but it is
    // reversed for right to left text.
    final (double visualPosition, double? secondaryVisualPosition) = switch (textDirection) {
      TextDirection.rtl when _secondaryTrackValue == null => (1.0 - controllerValue, null),
      TextDirection.rtl => (1.0 - controllerValue, 1.0 - _secondaryTrackValue!),
      TextDirection.ltr => (controllerValue, _secondaryTrackValue),
    };

    final Rect trackRect = _sliderTheme.trackShape!.getPreferredRect(
      parentBox: this,
      offset: offset,
      sliderTheme: _sliderTheme,
      isDiscrete: isDiscrete,
    );
    final double padding =
        isDiscrete || _sliderTheme.trackShape!.isRounded ? trackRect.height : 0.0;
    final double thumbPosition =
        isDiscrete
            ? trackRect.left + visualPosition * (trackRect.width - padding) + padding / 2
            : trackRect.left + visualPosition * trackRect.width;
    // Apply padding to trackRect.left and trackRect.right if the track height is
    // greater than the thumb radius to ensure the thumb is drawn within the track.
    final Size thumbPreferredSize = _sliderTheme.thumbShape!.getPreferredSize(
      isInteractive,
      isDiscrete,
    );
    final double thumbPadding = (padding > thumbPreferredSize.width / 2 ? padding / 2 : 0);
    final Offset thumbCenter = Offset(
      clampDouble(thumbPosition, trackRect.left + thumbPadding, trackRect.right - thumbPadding),
      trackRect.center.dy,
    );
    if (isInteractive) {
      final Size overlaySize = sliderTheme.overlayShape!.getPreferredSize(isInteractive, false);
      overlayRect = Rect.fromCircle(center: thumbCenter, radius: overlaySize.width / 2.0);
    }
    final Offset? secondaryOffset =
        (secondaryVisualPosition != null)
            ? Offset(
              trackRect.left + secondaryVisualPosition * trackRect.width,
              trackRect.center.dy,
            )
            : null;

    // If [Slider.year2023] is false, the thumb uses handle thumb shape and gapped track shape.
    // The handle width and track gap are adjusted when the thumb is pressed.
    double? thumbWidth = _sliderTheme.thumbSize?.resolve(<MaterialState>{})?.width;
    final double? thumbHeight = _sliderTheme.thumbSize?.resolve(<MaterialState>{})?.height;
    double? trackGap = _sliderTheme.trackGap;
    final double? pressedThumbWidth =
        _sliderTheme.thumbSize?.resolve(<MaterialState>{MaterialState.pressed})?.width;
    final double delta;
    if (_active && thumbWidth != null && pressedThumbWidth != null && trackGap != null) {
      delta = thumbWidth - pressedThumbWidth;
      if (thumbWidth > 0.0) {
        thumbWidth = pressedThumbWidth;
      }
      if (trackGap > 0.0) {
        trackGap = trackGap - delta / 2;
      }
    }

    _sliderTheme.trackShape!.paint(
      context,
      offset,
      parentBox: this,
      sliderTheme: _sliderTheme.copyWith(trackGap: trackGap),
      enableAnimation: _enableAnimation,
      textDirection: _textDirection,
      thumbCenter: thumbCenter,
      secondaryOffset: secondaryOffset,
      isDiscrete: isDiscrete,
      isEnabled: isInteractive,
    );

    if (!_overlayAnimation.isDismissed) {
      _sliderTheme.overlayShape!.paint(
        context,
        thumbCenter,
        activationAnimation: _overlayAnimation,
        enableAnimation: _enableAnimation,
        isDiscrete: isDiscrete,
        labelPainter: _labelPainter,
        parentBox: this,
        sliderTheme: _sliderTheme,
        textDirection: _textDirection,
        value: _value,
        textScaleFactor: _textScaleFactor,
        sizeWithOverflow: screenSize.isEmpty ? size : screenSize,
      );
    }

    if (isDiscrete) {
      final double tickMarkWidth =
          _sliderTheme.tickMarkShape!
              .getPreferredSize(isEnabled: isInteractive, sliderTheme: _sliderTheme)
              .width;
      final double adjustedTrackWidth = trackRect.width - padding;
      // If the tick marks would be too dense, don't bother painting them.
      if (adjustedTrackWidth / divisions! >= 3.0 * tickMarkWidth) {
        final double dy = trackRect.center.dy;
        for (int i = 0; i <= divisions!; i++) {
          final double value = i / divisions!;
          // The ticks are mapped to be within the track, so the tick mark width
          // must be subtracted from the track width.
          final double dx = trackRect.left + value * adjustedTrackWidth + padding / 2;
          final Offset tickMarkOffset = Offset(dx, dy);
          _sliderTheme.tickMarkShape!.paint(
            context,
            tickMarkOffset,
            parentBox: this,
            sliderTheme: _sliderTheme,
            enableAnimation: _enableAnimation,
            textDirection: _textDirection,
            thumbCenter: thumbCenter,
            isEnabled: isInteractive,
          );
        }
      }
    }

    if (isInteractive && label != null && !_valueIndicatorAnimation.isDismissed) {
      if (showValueIndicator) {
        _state.paintValueIndicator = (PaintingContext context, Offset offset) {
          if (attached && _labelPainter.text != null) {
            _sliderTheme.valueIndicatorShape!.paint(
              context,
              offset + thumbCenter,
              activationAnimation: _valueIndicatorAnimation,
              enableAnimation: _enableAnimation,
              isDiscrete: isDiscrete,
              labelPainter: _labelPainter,
              parentBox: this,
              sliderTheme: _sliderTheme,
              textDirection: _textDirection,
              value: _value,
              textScaleFactor: textScaleFactor,
              sizeWithOverflow: screenSize.isEmpty ? size : screenSize,
            );
          }
        };
      }
    }

    _sliderTheme.thumbShape!.paint(
      context,
      thumbCenter,
      activationAnimation: _overlayAnimation,
      enableAnimation: _enableAnimation,
      isDiscrete: isDiscrete,
      labelPainter: _labelPainter,
      parentBox: this,
      sliderTheme:
          thumbWidth != null && thumbHeight != null
              ? _sliderTheme.copyWith(
                thumbSize: MaterialStatePropertyAll<Size?>(Size(thumbWidth, thumbHeight)),
              )
              : _sliderTheme,
      textDirection: _textDirection,
      value: _value,
      textScaleFactor: textScaleFactor,
      sizeWithOverflow: screenSize.isEmpty ? size : screenSize,
    );
  }

  @override
  void describeSemanticsConfiguration(SemanticsConfiguration config) {
    super.describeSemanticsConfiguration(config);

    // The Slider widget has its own Focus widget with semantics information,
    // and we want that semantics node to collect the semantics information here
    // so that it's all in the same node: otherwise Talkback sees that the node
    // has focusable children, and it won't focus the Slider's Focus widget
    // because it thinks the Focus widget's node doesn't have anything to say
    // (which it doesn't, but this child does). Aggregating the semantic
    // information into one node means that Talkback will recognize that it has
    // something to say and focus it when it receives keyboard focus.
    // (See https://github.com/flutter/flutter/issues/57038 for context).
    config.isSemanticBoundary = false;

    config.isEnabled = isInteractive;
    config.textDirection = textDirection;
    if (isInteractive) {
      config.onIncrease = increaseAction;
      config.onDecrease = decreaseAction;
    }

    if (semanticFormatterCallback != null) {
      config.value = semanticFormatterCallback!(_state._lerp(value));
      config.increasedValue = semanticFormatterCallback!(
        _state._lerp(clampDouble(value + _semanticActionUnit, 0.0, 1.0)),
      );
      config.decreasedValue = semanticFormatterCallback!(
        _state._lerp(clampDouble(value - _semanticActionUnit, 0.0, 1.0)),
      );
    } else {
      config.value = '${(value * 100).round()}%';
      config.increasedValue =
          '${(clampDouble(value + _semanticActionUnit, 0.0, 1.0) * 100).round()}%';
      config.decreasedValue =
          '${(clampDouble(value - _semanticActionUnit, 0.0, 1.0) * 100).round()}%';
    }
  }

  double get _semanticActionUnit => divisions != null ? 1.0 / divisions! : _adjustmentUnit;

  void increaseAction() {
    if (isInteractive) {
      onChangeStart!(currentValue);
      final double increase = increaseValue();
      onChanged!(increase);
      onChangeEnd!(increase);
      if (!_state.mounted) {
        return;
      }
      _state.showValueIndicator();
    }
  }

  void decreaseAction() {
    if (isInteractive) {
      onChangeStart!(currentValue);
      final double decrease = decreaseValue();
      onChanged!(decrease);
      onChangeEnd!(decrease);
      if (!_state.mounted) {
        return;
      }
      _state.showValueIndicator();
    }
  }

  double get currentValue {
    return clampDouble(value, 0.0, 1.0);
  }

  double increaseValue() {
    return clampDouble(value + _semanticActionUnit, 0.0, 1.0);
  }

  double decreaseValue() {
    return clampDouble(value - _semanticActionUnit, 0.0, 1.0);
  }
}

class _AdjustSliderIntent extends Intent {
  const _AdjustSliderIntent({required this.type});

  const _AdjustSliderIntent.right() : type = _SliderAdjustmentType.right;

  const _AdjustSliderIntent.left() : type = _SliderAdjustmentType.left;

  const _AdjustSliderIntent.up() : type = _SliderAdjustmentType.up;

  const _AdjustSliderIntent.down() : type = _SliderAdjustmentType.down;

  final _SliderAdjustmentType type;
}

enum _SliderAdjustmentType { right, left, up, down }

class _ValueIndicatorRenderObjectWidget extends LeafRenderObjectWidget {
  const _ValueIndicatorRenderObjectWidget({required this.state});

  final _SliderState state;

  @override
  _RenderValueIndicator createRenderObject(BuildContext context) {
    return _RenderValueIndicator(state: state);
  }

  @override
  void updateRenderObject(BuildContext context, _RenderValueIndicator renderObject) {
    renderObject._state = state;
  }
}

class _RenderValueIndicator extends RenderBox with RelayoutWhenSystemFontsChangeMixin {
  _RenderValueIndicator({required _SliderState state}) : _state = state {
    _valueIndicatorAnimation = CurvedAnimation(
      parent: _state.valueIndicatorController,
      curve: Curves.fastOutSlowIn,
    );
  }
  late CurvedAnimation _valueIndicatorAnimation;
  _SliderState _state;

  @override
  bool get sizedByParent => true;

  @override
  void attach(PipelineOwner owner) {
    super.attach(owner);
    _valueIndicatorAnimation.addListener(markNeedsPaint);
    _state.positionController.addListener(markNeedsPaint);
  }

  @override
  void detach() {
    _valueIndicatorAnimation.removeListener(markNeedsPaint);
    _state.positionController.removeListener(markNeedsPaint);
    super.detach();
  }

  @override
  void paint(PaintingContext context, Offset offset) {
    _state.paintValueIndicator?.call(context, offset);
  }

  @override
  Size computeDryLayout(BoxConstraints constraints) {
    return constraints.smallest;
  }

  @override
  void dispose() {
    _valueIndicatorAnimation.dispose();
    super.dispose();
  }
}

class _SliderDefaultsM2 extends SliderThemeData {
  _SliderDefaultsM2(this.context) : super(trackHeight: 4.0);

  final BuildContext context;
  late final ColorScheme _colors = Theme.of(context).colorScheme;
  late final SliderThemeData sliderTheme = SliderTheme.of(context);

  @override
  Color? get activeTrackColor => _colors.primary;

  @override
  Color? get inactiveTrackColor => _colors.primary.withOpacity(0.24);

  @override
  Color? get secondaryActiveTrackColor => _colors.primary.withOpacity(0.54);

  @override
  Color? get disabledActiveTrackColor => _colors.onSurface.withOpacity(0.32);

  @override
  Color? get disabledInactiveTrackColor => _colors.onSurface.withOpacity(0.12);

  @override
  Color? get disabledSecondaryActiveTrackColor => _colors.onSurface.withOpacity(0.12);

  @override
  Color? get activeTickMarkColor => _colors.onPrimary.withOpacity(0.54);

  @override
  Color? get inactiveTickMarkColor => _colors.primary.withOpacity(0.54);

  @override
  Color? get disabledActiveTickMarkColor => _colors.onPrimary.withOpacity(0.12);

  @override
  Color? get disabledInactiveTickMarkColor => _colors.onSurface.withOpacity(0.12);

  @override
  Color? get thumbColor => _colors.primary;

  @override
  Color? get disabledThumbColor =>
      Color.alphaBlend(_colors.onSurface.withOpacity(.38), _colors.surface);

  @override
  Color? get overlayColor => _colors.primary.withOpacity(0.12);

  @override
  TextStyle? get valueIndicatorTextStyle =>
      Theme.of(context).textTheme.bodyLarge!.copyWith(color: _colors.onPrimary);

  @override
  Color? get valueIndicatorColor {
    if (sliderTheme.valueIndicatorShape is RoundedRectSliderValueIndicatorShape) {
      return _colors.inverseSurface;
    }
    return _colors.primary;
  }

  @override
  SliderComponentShape? get valueIndicatorShape => const RectangularSliderValueIndicatorShape();

  @override
  SliderComponentShape? get thumbShape => const RoundSliderThumbShape();

  @override
  SliderTrackShape? get trackShape => const RoundedRectSliderTrackShape();

  @override
  SliderComponentShape? get overlayShape => const RoundSliderOverlayShape();

  @override
  SliderTickMarkShape? get tickMarkShape => const RoundSliderTickMarkShape();
}

class _SliderDefaultsM3Year2023 extends SliderThemeData {
  _SliderDefaultsM3Year2023(this.context) : super(trackHeight: 4.0);

  final BuildContext context;
  late final ColorScheme _colors = Theme.of(context).colorScheme;

  @override
  Color? get activeTrackColor => _colors.primary;

  @override
  Color? get inactiveTrackColor => _colors.surfaceContainerHighest;

  @override
  Color? get secondaryActiveTrackColor => _colors.primary.withOpacity(0.54);

  @override
  Color? get disabledActiveTrackColor => _colors.onSurface.withOpacity(0.38);

  @override
  Color? get disabledInactiveTrackColor => _colors.onSurface.withOpacity(0.12);

  @override
  Color? get disabledSecondaryActiveTrackColor => _colors.onSurface.withOpacity(0.12);

  @override
  Color? get activeTickMarkColor => _colors.onPrimary.withOpacity(0.38);

  @override
  Color? get inactiveTickMarkColor => _colors.onSurfaceVariant.withOpacity(0.38);

  @override
  Color? get disabledActiveTickMarkColor => _colors.onSurface.withOpacity(0.38);

  @override
  Color? get disabledInactiveTickMarkColor => _colors.onSurface.withOpacity(0.38);

  @override
  Color? get thumbColor => _colors.primary;

  @override
  Color? get disabledThumbColor =>
      Color.alphaBlend(_colors.onSurface.withOpacity(0.38), _colors.surface);

  @override
  Color? get overlayColor => MaterialStateColor.resolveWith((Set<MaterialState> states) {
    if (states.contains(MaterialState.dragged)) {
      return _colors.primary.withOpacity(0.1);
    }
    if (states.contains(MaterialState.hovered)) {
      return _colors.primary.withOpacity(0.08);
    }
    if (states.contains(MaterialState.focused)) {
      return _colors.primary.withOpacity(0.1);
    }

    return Colors.transparent;
  });

  @override
  TextStyle? get valueIndicatorTextStyle =>
      Theme.of(context).textTheme.labelMedium!.copyWith(color: _colors.onPrimary);

  @override
  Color? get valueIndicatorColor => _colors.primary;

  @override
  SliderComponentShape? get valueIndicatorShape => const DropSliderValueIndicatorShape();

  @override
  SliderComponentShape? get thumbShape => const RoundSliderThumbShape();

  @override
  SliderTrackShape? get trackShape => const RoundedRectSliderTrackShape();

  @override
  SliderComponentShape? get overlayShape => const RoundSliderOverlayShape();

  @override
  SliderTickMarkShape? get tickMarkShape => const RoundSliderTickMarkShape();
}

// BEGIN GENERATED TOKEN PROPERTIES - Slider

// Do not edit by hand. The code between the "BEGIN GENERATED" and
// "END GENERATED" comments are generated from data in the Material
// Design token database by the script:
//   dev/tools/gen_defaults/bin/gen_defaults.dart.

// dart format off
class _SliderDefaultsM3 extends SliderThemeData {
  _SliderDefaultsM3(this.context)
    : super(trackHeight: 16.0);

  final BuildContext context;
  late final ColorScheme _colors = Theme.of(context).colorScheme;

  @override
  Color? get activeTrackColor => _colors.primary;

  @override
  Color? get inactiveTrackColor => _colors.secondaryContainer;

  @override
  Color? get secondaryActiveTrackColor => _colors.primary.withOpacity(0.54);

  @override
  Color? get disabledActiveTrackColor => _colors.onSurface.withOpacity(0.38);

  @override
  Color? get disabledInactiveTrackColor => _colors.onSurface.withOpacity(0.12);

  @override
  Color? get disabledSecondaryActiveTrackColor => _colors.onSurface.withOpacity(0.38);

  @override
  Color? get activeTickMarkColor => _colors.onPrimary.withOpacity(1.0);

  @override
  Color? get inactiveTickMarkColor => _colors.onSecondaryContainer.withOpacity(1.0);

  @override
  Color? get disabledActiveTickMarkColor => _colors.onInverseSurface;

  @override
  Color? get disabledInactiveTickMarkColor => _colors.onSurface;

  @override
  Color? get thumbColor => _colors.primary;

  @override
  Color? get disabledThumbColor => _colors.onSurface.withOpacity(0.38);

  @override
  Color? get overlayColor => MaterialStateColor.resolveWith((Set<MaterialState> states) {
    if (states.contains(MaterialState.dragged)) {
      return _colors.primary.withOpacity(0.1);
    }
    if (states.contains(MaterialState.hovered)) {
      return _colors.primary.withOpacity(0.08);
    }
    if (states.contains(MaterialState.focused)) {
      return _colors.primary.withOpacity(0.1);
    }

    return Colors.transparent;
  });

  @override
  TextStyle? get valueIndicatorTextStyle => Theme.of(context).textTheme.labelLarge!.copyWith(
    color: _colors.onInverseSurface,
  );

  @override
  Color? get valueIndicatorColor => _colors.inverseSurface;

  @override
  SliderComponentShape? get valueIndicatorShape => const RoundedRectSliderValueIndicatorShape();

  @override
  SliderComponentShape? get thumbShape => const HandleThumbShape();

  @override
  SliderTrackShape? get trackShape => const GappedSliderTrackShape();

  @override
  SliderComponentShape? get overlayShape => const RoundSliderOverlayShape();

  @override
  SliderTickMarkShape? get tickMarkShape => const RoundSliderTickMarkShape(tickMarkRadius: 4.0 / 2);

  @override
  MaterialStateProperty<Size?>? get thumbSize {
    return MaterialStateProperty.resolveWith((Set<MaterialState> states) {
      if (states.contains(MaterialState.disabled)) {
        return const Size(4.0, 44.0);
      }
      if (states.contains(MaterialState.hovered)) {
        return const Size(4.0, 44.0);
      }
      if (states.contains(MaterialState.focused)) {
        return const Size(2.0, 44.0);
      }
      if (states.contains(MaterialState.pressed)) {
        return const Size(2.0, 44.0);
      }
      return const Size(4.0, 44.0);
    });
  }

  @override
  double? get trackGap => 6.0;
}
// dart format on

// END GENERATED TOKEN PROPERTIES - Slider
