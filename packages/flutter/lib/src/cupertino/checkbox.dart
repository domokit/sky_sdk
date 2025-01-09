// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/material.dart';
///
/// @docImport 'slider.dart';
/// @docImport 'switch.dart';
library;

import 'package:flutter/foundation.dart';
import 'package:flutter/widgets.dart';

import 'colors.dart';
import 'constants.dart';
import 'theme.dart';

// Examples can assume:
// bool _throwShotAway = false;
// late StateSetter setState;

// Eyeballed from a checkbox on a physical Macbook Pro running macOS version 14.5.
const Color _kDisabledCheckColor = CupertinoDynamicColor.withBrightness(
  color: Color.fromARGB(64, 0, 0, 0),
  darkColor: Color.fromARGB(64, 255, 255, 255),
);
const Color _kDisabledBorderColor = CupertinoDynamicColor.withBrightness(
  color: Color.fromARGB(13, 0, 0, 0),
  darkColor: Color.fromARGB(13, 0, 0, 0),
);
const CupertinoDynamicColor _kDefaultBorderColor = CupertinoDynamicColor.withBrightness(
  color: Color.fromARGB(255, 209, 209, 214),
  darkColor: Color.fromARGB(50, 128, 128, 128),
);
const CupertinoDynamicColor _kDefaultFillColor = CupertinoDynamicColor.withBrightness(
  color: CupertinoColors.activeBlue,
  darkColor: Color.fromARGB(255, 50, 100, 215),
);
const Color _kDefaultCheckColor = CupertinoDynamicColor.withBrightness(
  color: CupertinoColors.white,
  darkColor: Color.fromARGB(255, 222, 232, 248),
);
const double _kPressedOverlayOpacity = 0.15;
// In dark mode, the fill color of a checkbox is an opacity gradient of the
// background color.
const List<double> _kDarkGradientOpacities = <double>[0.14, 0.29];
const List<double> _kDisabledDarkGradientOpacities = <double>[0.08, 0.14];

/// A macOS style checkbox.
///
/// The checkbox itself does not maintain any state. Instead, when the state of
/// the checkbox changes, the widget calls the [onChanged] callback. Most
/// widgets that use a checkbox will listen for the [onChanged] callback and
/// rebuild the checkbox with a new [value] to update the visual appearance of
/// the checkbox.
///
/// The checkbox can optionally display three values - true, false, and null -
/// if [tristate] is true. When [value] is null a dash is displayed. By default
/// [tristate] is false and the checkbox's [value] must be true or false.
///
/// In the Apple Human Interface Guidelines (HIG), checkboxes are encouraged for
/// use on macOS, but is silent about their use on iOS. If a multi-selection
/// component is needed on iOS, the HIG encourages the developer to use switches
/// ([CupertinoSwitch] in Flutter) instead, or to find a creative custom
/// solution.
///
/// {@tool dartpad}
/// This example shows a toggleable [CupertinoCheckbox].
///
/// ** See code in examples/api/lib/cupertino/checkbox/cupertino_checkbox.0.dart **
/// {@end-tool}
///
/// See also:
///
///  * [Checkbox], the Material Design equivalent.
///  * [CupertinoSwitch], a widget with semantics similar to [CupertinoCheckbox].
///  * [CupertinoSlider], for selecting a value in a range.
///  * <https://developer.apple.com/design/human-interface-guidelines/components/selection-and-input/toggles/>
class CupertinoCheckbox extends StatefulWidget {
  /// Creates a macOS-styled checkbox.
  ///
  /// The checkbox itself does not maintain any state. Instead, when the state of
  /// the checkbox changes, the widget calls the [onChanged] callback. Most
  /// widgets that use a checkbox will listen for the [onChanged] callback and
  /// rebuild the checkbox with a new [value] to update the visual appearance of
  /// the checkbox.
  ///
  /// The following arguments are required:
  ///
  /// * [value], which determines whether the checkbox is checked. The [value]
  ///   can only be null if [tristate] is true.
  /// * [onChanged], which is called when the value of the checkbox should
  ///   change. It can be set to null to disable the checkbox.
  const CupertinoCheckbox({
    super.key,
    required this.value,
    this.tristate = false,
    required this.onChanged,
    this.mouseCursor,
    this.activeColor,
    @Deprecated(
      'Use fillColor instead. '
      'fillColor now manages the background color in all states. '
      'This feature was deprecated after v3.24.0-0.2.pre.',
    )
    this.inactiveColor,
    this.fillColor,
    this.checkColor,
    this.focusColor,
    this.focusNode,
    this.autofocus = false,
    this.side,
    this.shape,
    this.semanticLabel,
  }) : assert(tristate || value != null);

  /// Whether this checkbox is checked.
  ///
  /// When [tristate] is true, a value of null corresponds to the mixed state.
  /// When [tristate] is false, this value must not be null. This is asserted in
  /// debug mode.
  final bool? value;

  /// Called when the value of the checkbox should change.
  ///
  /// The checkbox passes the new value to the callback but does not actually
  /// change state until the parent widget rebuilds the checkbox with the new
  /// value.
  ///
  /// If this callback is null, the checkbox will be displayed as disabled
  /// and will not respond to input gestures.
  ///
  /// When the checkbox is tapped, if [tristate] is false (the default) then
  /// the [onChanged] callback will be applied to `!value`. If [tristate] is
  /// true this callback cycle from false to true to null and back to false
  /// again.
  ///
  /// The callback provided to [onChanged] should update the state of the parent
  /// [StatefulWidget] using the [State.setState] method, so that the parent
  /// gets rebuilt; for example:
  ///
  /// ```dart
  /// CupertinoCheckbox(
  ///   value: _throwShotAway,
  ///   onChanged: (bool? newValue) {
  ///     setState(() {
  ///       _throwShotAway = newValue!;
  ///     });
  ///   },
  /// )
  /// ```
  final ValueChanged<bool?>? onChanged;

  /// The cursor for a mouse pointer when it enters or is hovering over the
  /// widget.
  ///
  /// If [mouseCursor] is a [WidgetStateMouseCursor],
  /// [WidgetStateProperty.resolve] is used for the following [WidgetState]s:
  ///
  ///  * [WidgetState.selected].
  ///  * [WidgetState.focused].
  ///  * [WidgetState.disabled].
  ///
  /// When [value] is null and [tristate] is true, [WidgetState.selected] is
  /// included as a state.
  ///
  /// If null, then [SystemMouseCursors.basic] is used when this checkbox is
  /// disabled. When the checkbox is enabled, [SystemMouseCursors.click] is used
  /// on Web, and [SystemMouseCursors.basic] is used on other platforms.
  ///
  /// See also:
  ///
  ///  * [WidgetStateMouseCursor], a [MouseCursor] that implements
  ///    [WidgetStateProperty] which is used in APIs that need to accept
  ///    either a [MouseCursor] or a [WidgetStateProperty].
  final MouseCursor? mouseCursor;

  /// The color to use when this checkbox is checked.
  ///
  /// If [fillColor] returns a non-null color in the [WidgetState.selected]
  /// state, [fillColor] will be used instead of [activeColor].
  ///
  /// Defaults to [CupertinoColors.activeBlue].
  final Color? activeColor;

  /// {@template flutter.cupertino.CupertinoCheckbox.fillColor}
  /// The color used to fill this checkbox.
  ///
  /// Resolves in the following states:
  ///  * [WidgetState.selected].
  ///  * [WidgetState.hovered].
  ///  * [WidgetState.focused].
  ///  * [WidgetState.disabled].
  ///
  /// {@tool snippet}
  /// This example resolves the [fillColor] based on the current [WidgetState]
  /// of the [CupertinoCheckbox], providing a different [Color] when it is
  /// [WidgetState.disabled].
  ///
  /// ```dart
  /// CupertinoCheckbox(
  ///   value: true,
  ///   onChanged: (_){},
  ///   fillColor: WidgetStateProperty.resolveWith<Color>((Set<WidgetState> states) {
  ///     if (states.contains(WidgetState.disabled)) {
  ///       return Colors.orange.withOpacity(.32);
  ///     }
  ///     return Colors.orange;
  ///   })
  /// )
  /// ```
  /// {@end-tool}
  /// {@endtemplate}
  ///
  /// If [fillColor] resolves to null for the requested state, then the fill color
  /// falls back to [activeColor] if the state includes [WidgetState.selected],
  /// [CupertinoColors.white] at 50% opacity if checkbox is disabled,
  /// and [CupertinoColors.white] otherwise.
  final WidgetStateProperty<Color?>? fillColor;

  /// The color used if the checkbox is inactive.
  ///
  /// Currently [inactiveColor] is not used. Instead, [fillColor] controls the
  /// color of the background in all states, including when unselected.
  @Deprecated(
    'Use fillColor instead. '
    'fillColor now manages the background color in all states. '
    'This feature was deprecated after v3.24.0-0.2.pre.',
  )
  final Color? inactiveColor;

  /// The color to use for the check icon when this checkbox is checked.
  ///
  /// If null, then the value of [CupertinoColors.white] is used if the checkbox
  /// is enabled. If the checkbox is disabled, a grey-black color is used.
  final Color? checkColor;

  /// If true, the checkbox's [value] can be true, false, or null.
  ///
  /// [CupertinoCheckbox] displays a dash when its value is null.
  ///
  /// When a tri-state checkbox ([tristate] is true) is tapped, its [onChanged]
  /// callback will be applied to true if the current value is false, to null if
  /// value is true, and to false if value is null (i.e. it cycles through false
  /// => true => null => false when tapped).
  ///
  /// If tristate is false (the default), [value] must not be null, and
  /// [onChanged] will only toggle between true and false.
  final bool tristate;

  /// The color for the checkbox's border shadow when it has the input focus.
  ///
  /// If null, then a paler form of the [activeColor] will be used.
  final Color? focusColor;

  /// {@macro flutter.widgets.Focus.focusNode}
  final FocusNode? focusNode;

  /// {@macro flutter.widgets.Focus.autofocus}
  final bool autofocus;

  /// The color and width of the checkbox's border.
  ///
  /// This property can be a [WidgetStateBorderSide] that can
  /// specify different border color and widths depending on the
  /// checkbox's state.
  ///
  /// Resolves in the following states:
  ///  * [WidgetState.pressed].
  ///  * [WidgetState.selected].
  ///  * [WidgetState.hovered].
  ///  * [WidgetState.focused].
  ///  * [WidgetState.disabled].
  ///  * [WidgetState.error].
  ///
  /// If this property is not a [WidgetStateBorderSide] and it is
  /// non-null, then it is only rendered when the checkbox's value is
  /// false. The difference in interpretation is for backwards
  /// compatibility.
  ///
  /// If this property is null and the checkbox's value is false, then the side
  /// defaults to a one pixel wide grey-black border.
  final BorderSide? side;

  /// The shape of the checkbox.
  ///
  /// If this property is null then the shape defaults to a
  /// [RoundedRectangleBorder] with a circular corner radius of 4.0.
  final OutlinedBorder? shape;

  /// The semantic label for the checkbox that will be announced by screen readers.
  ///
  /// This is announced by assistive technologies (e.g TalkBack/VoiceOver).
  ///
  /// This label does not show in the UI.
  final String? semanticLabel;

  /// The width of a checkbox widget.
  static const double width = 14.0;

  @override
  State<CupertinoCheckbox> createState() => _CupertinoCheckboxState();
}

class _CupertinoCheckboxState extends State<CupertinoCheckbox>
    with TickerProviderStateMixin, ToggleableStateMixin {
  final _CheckboxPainter _painter = _CheckboxPainter();
  bool? _previousValue;

  bool focused = false;

  @override
  void initState() {
    super.initState();
    _previousValue = widget.value;
  }

  @override
  void didUpdateWidget(CupertinoCheckbox oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (oldWidget.value != widget.value) {
      _previousValue = oldWidget.value;
    }
  }

  @override
  void dispose() {
    _painter.dispose();
    super.dispose();
  }

  @override
  ValueChanged<bool?>? get onChanged => widget.onChanged;

  @override
  bool get tristate => widget.tristate;

  @override
  bool? get value => widget.value;

  WidgetStateProperty<Color> get _defaultFillColor {
    return WidgetStateProperty.resolveWith((Set<WidgetState> states) {
      if (states.contains(WidgetState.disabled)) {
        return CupertinoColors.white.withOpacity(0.5);
      }
      if (states.contains(WidgetState.selected)) {
        return widget.activeColor ?? CupertinoDynamicColor.resolve(_kDefaultFillColor, context);
      }
      return CupertinoColors.white;
    });
  }

  WidgetStateProperty<Color> get _defaultCheckColor {
    return WidgetStateProperty.resolveWith((Set<WidgetState> states) {
      if (states.contains(WidgetState.disabled) && states.contains(WidgetState.selected)) {
        return widget.checkColor ?? CupertinoDynamicColor.resolve(_kDisabledCheckColor, context);
      }
      if (states.contains(WidgetState.selected)) {
        return widget.checkColor ?? CupertinoDynamicColor.resolve(_kDefaultCheckColor, context);
      }
      return CupertinoColors.white;
    });
  }

  WidgetStateProperty<BorderSide> get _defaultSide {
    return WidgetStateProperty.resolveWith((Set<WidgetState> states) {
      if ((states.contains(WidgetState.selected) || states.contains(WidgetState.focused)) &&
          !states.contains(WidgetState.disabled)) {
        return const BorderSide(width: 0.0, color: CupertinoColors.transparent);
      }
      if (states.contains(WidgetState.disabled)) {
        return BorderSide(color: CupertinoDynamicColor.resolve(_kDisabledBorderColor, context));
      }
      return BorderSide(color: CupertinoDynamicColor.resolve(_kDefaultBorderColor, context));
    });
  }

  BorderSide? _resolveSide(BorderSide? side, Set<WidgetState> states) {
    if (side is WidgetStateBorderSide) {
      return WidgetStateProperty.resolveAs<BorderSide?>(side, states);
    }
    if (!states.contains(WidgetState.selected)) {
      return side;
    }
    return null;
  }

  @override
  Widget build(BuildContext context) {
    // Colors need to be resolved in selected and non selected states separately.
    // The `states` getter constructs a new set every time, making it safe to edit in place.
    final Set<WidgetState> activeStates = states..add(WidgetState.selected);
    final Set<WidgetState> inactiveStates = states..remove(WidgetState.selected);

    // Since the states getter always makes a new set, make a copy to use
    // throughout the lifecycle of this build method.
    final Set<WidgetState> currentStates = states;

    final Color effectiveActiveColor =
        widget.fillColor?.resolve(activeStates) ?? _defaultFillColor.resolve(activeStates);

    final Color effectiveInactiveColor =
        widget.fillColor?.resolve(inactiveStates) ?? _defaultFillColor.resolve(inactiveStates);

    final BorderSide effectiveBorderSide =
        _resolveSide(widget.side, currentStates) ?? _defaultSide.resolve(currentStates);

    final Color effectiveFocusOverlayColor =
        widget.focusColor ??
        HSLColor.fromColor(effectiveActiveColor.withOpacity(kCupertinoFocusColorOpacity))
            .withLightness(kCupertinoFocusColorBrightness)
            .withSaturation(kCupertinoFocusColorSaturation)
            .toColor();

    final WidgetStateProperty<MouseCursor> effectiveMouseCursor =
        WidgetStateProperty.resolveWith<MouseCursor>((Set<WidgetState> states) {
          return WidgetStateProperty.resolveAs<MouseCursor?>(widget.mouseCursor, states) ??
              (kIsWeb && !states.contains(WidgetState.disabled)
                  ? SystemMouseCursors.click
                  : SystemMouseCursors.basic);
        });

    return Semantics(
      label: widget.semanticLabel,
      checked: widget.value ?? false,
      mixed: widget.tristate ? widget.value == null : null,
      child: buildToggleable(
        mouseCursor: effectiveMouseCursor,
        focusNode: widget.focusNode,
        autofocus: widget.autofocus,
        size: const Size.square(kMinInteractiveDimensionCupertino),
        painter:
            _painter
              ..position = position
              ..reaction = reaction
              ..focusColor = effectiveFocusOverlayColor
              ..downPosition = downPosition
              ..isFocused = currentStates.contains(WidgetState.focused)
              ..isHovered = currentStates.contains(WidgetState.hovered)
              ..activeColor = effectiveActiveColor
              ..inactiveColor = effectiveInactiveColor
              ..checkColor = _defaultCheckColor.resolve(currentStates)
              ..value = value
              ..previousValue = _previousValue
              ..isActive = widget.onChanged != null
              ..shape =
                  widget.shape ?? RoundedRectangleBorder(borderRadius: BorderRadius.circular(4.0))
              ..side = effectiveBorderSide
              ..brightness = CupertinoTheme.of(context).brightness,
      ),
    );
  }
}

class _CheckboxPainter extends ToggleablePainter {
  Color get checkColor => _checkColor!;
  Color? _checkColor;
  set checkColor(Color value) {
    if (_checkColor == value) {
      return;
    }
    _checkColor = value;
    notifyListeners();
  }

  bool? get value => _value;
  bool? _value;
  set value(bool? value) {
    if (_value == value) {
      return;
    }
    _value = value;
    notifyListeners();
  }

  bool? get previousValue => _previousValue;
  bool? _previousValue;
  set previousValue(bool? value) {
    if (_previousValue == value) {
      return;
    }
    _previousValue = value;
    notifyListeners();
  }

  OutlinedBorder get shape => _shape!;
  OutlinedBorder? _shape;
  set shape(OutlinedBorder value) {
    if (_shape == value) {
      return;
    }
    _shape = value;
    notifyListeners();
  }

  BorderSide get side => _side!;
  BorderSide? _side;
  set side(BorderSide value) {
    if (_side == value) {
      return;
    }
    _side = value;
    notifyListeners();
  }

  Brightness? get brightness => _brightness;
  Brightness? _brightness;
  set brightness(Brightness? value) {
    if (_brightness == value) {
      return;
    }
    _brightness = value;
    notifyListeners();
  }

  Rect _outerRectAt(Offset origin) {
    const double size = CupertinoCheckbox.width;
    final Rect rect = Rect.fromLTWH(origin.dx, origin.dy, size, size);
    return rect;
  }

  // The checkbox's border color if value == false, or its fill color when
  // value == true or null.
  Color _colorAt(bool value) {
    return value && isActive ? activeColor : inactiveColor;
  }

  // White stroke used to paint the check and dash.
  Paint _createStrokePaint() {
    return Paint()
      ..color = checkColor
      ..style = PaintingStyle.stroke
      ..strokeWidth = 2.0
      ..strokeCap = StrokeCap.round;
  }

  // Draw a gradient from the top to the bottom of the checkbox.
  void _drawFillGradient(Canvas canvas, Rect outer, Color topColor, Color bottomColor) {
    final LinearGradient fillGradient = LinearGradient(
      begin: Alignment.topCenter,
      end: Alignment.bottomCenter,
      // Eyeballed from a checkbox on a physical Macbook Pro running macOS version 14.5.
      colors: <Color>[topColor, bottomColor],
    );
    final Paint gradientPaint = Paint()..shader = fillGradient.createShader(outer);
    canvas.drawPath(shape.getOuterPath(outer), gradientPaint);
  }

  void _drawBox(Canvas canvas, Rect outer, Paint paint, BorderSide? side, bool value) {
    // Draw a gradient in dark mode except when the checkbox is enabled and checked.
    if (brightness == Brightness.dark && !(isActive && value)) {
      _drawFillGradient(
        canvas,
        outer,
        paint.color.withOpacity(
          isActive ? _kDarkGradientOpacities[0] : _kDisabledDarkGradientOpacities[0],
        ),
        paint.color.withOpacity(
          isActive ? _kDarkGradientOpacities[1] : _kDisabledDarkGradientOpacities[1],
        ),
      );
    } else {
      canvas.drawPath(shape.getOuterPath(outer), paint);
    }
    if (side != null) {
      shape.copyWith(side: side).paint(canvas, outer);
    }
  }

  void _drawCheck(Canvas canvas, Offset origin, Paint paint) {
    final Path path = Path();
    // The ratios for the offsets below were found from looking at the checkbox
    // examples on in the HIG docs. The distance from the needed point to the
    // edge was measured, then divided by the total width.
    const Offset start = Offset(CupertinoCheckbox.width * 0.22, CupertinoCheckbox.width * 0.54);
    const Offset mid = Offset(CupertinoCheckbox.width * 0.40, CupertinoCheckbox.width * 0.75);
    const Offset end = Offset(CupertinoCheckbox.width * 0.78, CupertinoCheckbox.width * 0.25);
    path.moveTo(origin.dx + start.dx, origin.dy + start.dy);
    path.lineTo(origin.dx + mid.dx, origin.dy + mid.dy);
    path.moveTo(origin.dx + mid.dx, origin.dy + mid.dy);
    path.lineTo(origin.dx + end.dx, origin.dy + end.dy);
    canvas.drawPath(path, paint);
  }

  void _drawDash(Canvas canvas, Offset origin, Paint paint) {
    // From measuring the checkbox example in the HIG docs, the dash was found
    // to be half the total width, centered in the middle.
    const Offset start = Offset(CupertinoCheckbox.width * 0.25, CupertinoCheckbox.width * 0.5);
    const Offset end = Offset(CupertinoCheckbox.width * 0.75, CupertinoCheckbox.width * 0.5);
    canvas.drawLine(origin + start, origin + end, paint);
  }

  @override
  void paint(Canvas canvas, Size size) {
    final Paint strokePaint = _createStrokePaint();
    final Offset origin = size / 2.0 - const Size.square(CupertinoCheckbox.width) / 2.0 as Offset;
    final Rect outer = _outerRectAt(origin);
    final Paint paint = Paint()..color = _colorAt(value ?? true);

    switch (value) {
      case false:
        _drawBox(canvas, outer, paint, side, value ?? true);
      case true:
        _drawBox(canvas, outer, paint, side, value ?? true);
        _drawCheck(canvas, origin, strokePaint);
      case null:
        _drawBox(canvas, outer, paint, side, value ?? true);
        _drawDash(canvas, origin, strokePaint);
    }
    // The checkbox's opacity changes when pressed.
    if (downPosition != null) {
      final Paint pressedPaint =
          Paint()
            ..color =
                brightness == Brightness.light
                    ? CupertinoColors.black.withOpacity(_kPressedOverlayOpacity)
                    : CupertinoColors.white.withOpacity(_kPressedOverlayOpacity);
      canvas.drawPath(shape.getOuterPath(outer), pressedPaint);
    }
    if (isFocused) {
      final Rect focusOuter = outer.inflate(1);
      final Paint borderPaint =
          Paint()
            ..color = focusColor
            ..style = PaintingStyle.stroke
            ..strokeWidth = 3.5;
      _drawBox(canvas, focusOuter, borderPaint, side, value ?? true);
    }
  }
}
