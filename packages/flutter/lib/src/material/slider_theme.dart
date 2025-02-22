// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'color_scheme.dart';
/// @docImport 'range_slider.dart';
/// @docImport 'text_theme.dart';
library;

import 'dart:ui' show lerpDouble;

import 'package:flutter/foundation.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/widgets.dart';

import 'colors.dart';
import 'material_state.dart';
import 'range_slider_parts.dart';
import 'slider.dart';
import 'slider_parts.dart';
import 'theme.dart';

/// Applies a slider theme to descendant [Slider] widgets.
///
/// A slider theme describes the colors and shape choices of the slider
/// components.
///
/// Descendant widgets obtain the current theme's [SliderThemeData] object using
/// [SliderTheme.of]. When a widget uses [SliderTheme.of], it is automatically
/// rebuilt if the theme later changes.
///
/// The slider is as big as the largest of
/// the [SliderComponentShape.getPreferredSize] of the thumb shape,
/// the [SliderComponentShape.getPreferredSize] of the overlay shape,
/// and the [SliderTickMarkShape.getPreferredSize] of the tick mark shape.
///
/// See also:
///
///  * [SliderThemeData], which describes the actual configuration of a slider
///    theme.
///  * [SliderComponentShape], which can be used to create custom shapes for
///    the [Slider]'s thumb, overlay, and value indicator and the
///    [RangeSlider]'s overlay.
///  * [SliderTrackShape], which can be used to create custom shapes for the
///    [Slider]'s track.
///  * [SliderTickMarkShape], which can be used to create custom shapes for the
///    [Slider]'s tick marks.
///  * [RangeSliderThumbShape], which can be used to create custom shapes for
///    the [RangeSlider]'s thumb.
///  * [RangeSliderValueIndicatorShape], which can be used to create custom
///    shapes for the [RangeSlider]'s value indicator.
///  * [RangeSliderTrackShape], which can be used to create custom shapes for
///    the [RangeSlider]'s track.
///  * [RangeSliderTickMarkShape], which can be used to create custom shapes for
///    the [RangeSlider]'s tick marks.
class SliderTheme extends InheritedTheme {
  /// Applies the given theme [data] to [child].
  const SliderTheme({super.key, required this.data, required super.child});

  /// Specifies the color and shape values for descendant slider widgets.
  final SliderThemeData data;

  /// Returns the data from the closest [SliderTheme] instance that encloses
  /// the given context.
  ///
  /// Defaults to the ambient [ThemeData.sliderTheme] if there is no
  /// [SliderTheme] in the given build context.
  ///
  /// {@tool snippet}
  ///
  /// ```dart
  /// class Launch extends StatefulWidget {
  ///   const Launch({super.key});
  ///
  ///   @override
  ///   State createState() => LaunchState();
  /// }
  ///
  /// class LaunchState extends State<Launch> {
  ///   double _rocketThrust = 0;
  ///
  ///   @override
  ///   Widget build(BuildContext context) {
  ///     return SliderTheme(
  ///       data: SliderTheme.of(context).copyWith(activeTrackColor: const Color(0xff804040)),
  ///       child: Slider(
  ///         onChanged: (double value) { setState(() { _rocketThrust = value; }); },
  ///         value: _rocketThrust,
  ///       ),
  ///     );
  ///   }
  /// }
  /// ```
  /// {@end-tool}
  ///
  /// See also:
  ///
  ///  * [SliderThemeData], which describes the actual configuration of a slider
  ///    theme.
  static SliderThemeData of(BuildContext context) {
    final SliderTheme? inheritedTheme = context.dependOnInheritedWidgetOfExactType<SliderTheme>();
    return inheritedTheme != null ? inheritedTheme.data : Theme.of(context).sliderTheme;
  }

  @override
  Widget wrap(BuildContext context, Widget child) {
    return SliderTheme(data: data, child: child);
  }

  @override
  bool updateShouldNotify(SliderTheme oldWidget) => data != oldWidget.data;
}

/// Describes the conditions under which the value indicator on a [Slider]
/// will be shown. Used with [SliderThemeData.showValueIndicator].
///
/// See also:
///
///  * [Slider], a Material Design slider widget.
///  * [SliderThemeData], which describes the actual configuration of a slider
///    theme.
enum ShowValueIndicator {
  /// The value indicator will only be shown for discrete sliders (sliders
  /// where [Slider.divisions] is non-null).
  onlyForDiscrete,

  /// The value indicator will only be shown for continuous sliders (sliders
  /// where [Slider.divisions] is null).
  onlyForContinuous,

  /// The value indicator will be shown for all types of sliders.
  always,

  /// The value indicator will never be shown.
  never,
}

/// Identifier for a thumb.
///
/// There are 2 thumbs in a [RangeSlider], [start] and [end].
///
/// For [TextDirection.ltr], the [start] thumb is the left-most thumb and the
/// [end] thumb is the right-most thumb. For [TextDirection.rtl] the [start]
/// thumb is the right-most thumb, and the [end] thumb is the left-most thumb.
enum Thumb {
  /// Left-most thumb for [TextDirection.ltr], otherwise, right-most thumb.
  start,

  /// Right-most thumb for [TextDirection.ltr], otherwise, left-most thumb.
  end,
}

/// Holds the color, shape, and typography values for a Material Design slider
/// theme.
///
/// Use this class to configure a [SliderTheme] widget, or to set the
/// [ThemeData.sliderTheme] for a [Theme] widget.
///
/// To obtain the current ambient slider theme, use [SliderTheme.of].
///
/// This theme is for both the [Slider] and the [RangeSlider]. The properties
/// that are only for the [Slider] are: [tickMarkShape], [thumbShape],
/// [trackShape], and [valueIndicatorShape]. The properties that are only for
/// the [RangeSlider] are [rangeTickMarkShape], [rangeThumbShape],
/// [rangeTrackShape], [rangeValueIndicatorShape],
/// [overlappingShapeStrokeColor], [minThumbSeparation], and [thumbSelector].
/// All other properties are used by both the [Slider] and the [RangeSlider].
///
/// The parts of a slider are:
///
///  * The "thumb", which is a shape that slides horizontally when the user
///    drags it.
///  * The "track", which is the line that the slider thumb slides along.
///  * The "tick marks", which are regularly spaced marks that are drawn when
///    using discrete divisions.
///  * The "value indicator", which appears when the user is dragging the thumb
///    to indicate the value being selected.
///  * The "overlay", which appears around the thumb, and is shown when the
///    thumb is pressed, focused, or hovered. It is painted underneath the
///    thumb, so it must extend beyond the bounds of the thumb itself to
///    actually be visible.
///  * The "active" side of the slider is the side between the thumb and the
///    minimum value.
///  * The "inactive" side of the slider is the side between the thumb and the
///    maximum value.
///  * The [Slider] is disabled when it is not accepting user input. See
///    [Slider] for details on when this happens.
///
/// The thumb, track, tick marks, value indicator, and overlay can be customized
/// by creating subclasses of [SliderTrackShape],
/// [SliderComponentShape], and/or [SliderTickMarkShape]. See
/// [RoundSliderThumbShape], [RectangularSliderTrackShape],
/// [RoundSliderTickMarkShape], [RectangularSliderValueIndicatorShape], and
/// [RoundSliderOverlayShape] for examples.
///
/// The track painting can be skipped by specifying 0 for [trackHeight].
/// The thumb painting can be skipped by specifying
/// [SliderComponentShape.noThumb] for [SliderThemeData.thumbShape].
/// The overlay painting can be skipped by specifying
/// [SliderComponentShape.noOverlay] for [SliderThemeData.overlayShape].
/// The tick mark painting can be skipped by specifying
/// [SliderTickMarkShape.noTickMark] for [SliderThemeData.tickMarkShape].
/// The value indicator painting can be skipped by specifying the
/// appropriate [ShowValueIndicator] for [SliderThemeData.showValueIndicator].
///
/// See also:
///
///  * [SliderTheme] widget, which can override the slider theme of its
///    children.
///  * [Theme] widget, which performs a similar function to [SliderTheme],
///    but for overall themes.
///  * [ThemeData], which has a default [SliderThemeData].
///  * [SliderComponentShape], which can be used to create custom shapes for
///    the [Slider]'s thumb, overlay, and value indicator and the
///    [RangeSlider]'s overlay.
///  * [SliderTrackShape], which can be used to create custom shapes for the
///    [Slider]'s track.
///  * [SliderTickMarkShape], which can be used to create custom shapes for the
///    [Slider]'s tick marks.
///  * [RangeSliderThumbShape], which can be used to create custom shapes for
///    the [RangeSlider]'s thumb.
///  * [RangeSliderValueIndicatorShape], which can be used to create custom
///    shapes for the [RangeSlider]'s value indicator.
///  * [RangeSliderTrackShape], which can be used to create custom shapes for
///    the [RangeSlider]'s track.
///  * [RangeSliderTickMarkShape], which can be used to create custom shapes for
///    the [RangeSlider]'s tick marks.
@immutable
class SliderThemeData with Diagnosticable {
  /// Create a [SliderThemeData] given a set of exact values.
  ///
  /// This will rarely be used directly. It is used by [lerp] to
  /// create intermediate themes based on two themes.
  ///
  /// The simplest way to create a SliderThemeData is to use
  /// [copyWith] on the one you get from [SliderTheme.of], or create an
  /// entirely new one with [SliderThemeData.fromPrimaryColors].
  ///
  /// {@tool snippet}
  ///
  /// ```dart
  /// class Blissful extends StatefulWidget {
  ///   const Blissful({super.key});
  ///
  ///   @override
  ///   State createState() => BlissfulState();
  /// }
  ///
  /// class BlissfulState extends State<Blissful> {
  ///   double _bliss = 0;
  ///
  ///   @override
  ///   Widget build(BuildContext context) {
  ///     return SliderTheme(
  ///       data: SliderTheme.of(context).copyWith(activeTrackColor: const Color(0xff404080)),
  ///       child: Slider(
  ///         onChanged: (double value) { setState(() { _bliss = value; }); },
  ///         value: _bliss,
  ///       ),
  ///     );
  ///   }
  /// }
  /// ```
  /// {@end-tool}
  const SliderThemeData({
    this.trackHeight,
    this.activeTrackColor,
    this.inactiveTrackColor,
    this.secondaryActiveTrackColor,
    this.disabledActiveTrackColor,
    this.disabledInactiveTrackColor,
    this.disabledSecondaryActiveTrackColor,
    this.activeTickMarkColor,
    this.inactiveTickMarkColor,
    this.disabledActiveTickMarkColor,
    this.disabledInactiveTickMarkColor,
    this.thumbColor,
    this.overlappingShapeStrokeColor,
    this.disabledThumbColor,
    this.overlayColor,
    this.valueIndicatorColor,
    this.valueIndicatorStrokeColor,
    this.overlayShape,
    this.tickMarkShape,
    this.thumbShape,
    this.trackShape,
    this.valueIndicatorShape,
    this.rangeTickMarkShape,
    this.rangeThumbShape,
    this.rangeTrackShape,
    this.rangeValueIndicatorShape,
    this.showValueIndicator,
    this.valueIndicatorTextStyle,
    this.minThumbSeparation,
    this.thumbSelector,
    this.mouseCursor,
    this.allowedInteraction,
    this.padding,
    this.thumbSize,
    this.trackGap,
    @Deprecated(
      'Set this flag to false to opt into the 2024 slider appearance. Defaults to true. '
      'In the future, this flag will default to false. Use SliderThemeData to customize individual properties. '
      'This feature was deprecated after v3.27.0-0.2.pre.',
    )
    this.year2023,
  });

  /// Generates a SliderThemeData from three main colors.
  ///
  /// Usually these are the primary, dark and light colors from
  /// a [ThemeData].
  ///
  /// The opacities of these colors will be overridden with the Material Design
  /// defaults when assigning them to the slider theme component colors.
  ///
  /// This is used to generate the default slider theme for a [ThemeData].
  factory SliderThemeData.fromPrimaryColors({
    required Color primaryColor,
    required Color primaryColorDark,
    required Color primaryColorLight,
    required TextStyle valueIndicatorTextStyle,
  }) {
    // These are Material Design defaults, and are used to derive
    // component Colors (with opacity) from base colors.
    const int activeTrackAlpha = 0xff;
    const int inactiveTrackAlpha = 0x3d; // 24% opacity
    const int secondaryActiveTrackAlpha = 0x8a; // 54% opacity
    const int disabledActiveTrackAlpha = 0x52; // 32% opacity
    const int disabledInactiveTrackAlpha = 0x1f; // 12% opacity
    const int disabledSecondaryActiveTrackAlpha = 0x1f; // 12% opacity
    const int activeTickMarkAlpha = 0x8a; // 54% opacity
    const int inactiveTickMarkAlpha = 0x8a; // 54% opacity
    const int disabledActiveTickMarkAlpha = 0x1f; // 12% opacity
    const int disabledInactiveTickMarkAlpha = 0x1f; // 12% opacity
    const int thumbAlpha = 0xff;
    const int disabledThumbAlpha = 0x52; // 32% opacity
    const int overlayAlpha = 0x1f; // 12% opacity
    const int valueIndicatorAlpha = 0xff;

    return SliderThemeData(
      trackHeight: 2.0,
      activeTrackColor: primaryColor.withAlpha(activeTrackAlpha),
      inactiveTrackColor: primaryColor.withAlpha(inactiveTrackAlpha),
      secondaryActiveTrackColor: primaryColor.withAlpha(secondaryActiveTrackAlpha),
      disabledActiveTrackColor: primaryColorDark.withAlpha(disabledActiveTrackAlpha),
      disabledInactiveTrackColor: primaryColorDark.withAlpha(disabledInactiveTrackAlpha),
      disabledSecondaryActiveTrackColor: primaryColorDark.withAlpha(
        disabledSecondaryActiveTrackAlpha,
      ),
      activeTickMarkColor: primaryColorLight.withAlpha(activeTickMarkAlpha),
      inactiveTickMarkColor: primaryColor.withAlpha(inactiveTickMarkAlpha),
      disabledActiveTickMarkColor: primaryColorLight.withAlpha(disabledActiveTickMarkAlpha),
      disabledInactiveTickMarkColor: primaryColorDark.withAlpha(disabledInactiveTickMarkAlpha),
      thumbColor: primaryColor.withAlpha(thumbAlpha),
      overlappingShapeStrokeColor: Colors.white,
      disabledThumbColor: primaryColorDark.withAlpha(disabledThumbAlpha),
      overlayColor: primaryColor.withAlpha(overlayAlpha),
      valueIndicatorColor: primaryColor.withAlpha(valueIndicatorAlpha),
      valueIndicatorStrokeColor: primaryColor.withAlpha(valueIndicatorAlpha),
      overlayShape: const RoundSliderOverlayShape(),
      tickMarkShape: const RoundSliderTickMarkShape(),
      thumbShape: const RoundSliderThumbShape(),
      trackShape: const RoundedRectSliderTrackShape(),
      valueIndicatorShape: const PaddleSliderValueIndicatorShape(),
      rangeTickMarkShape: const RoundRangeSliderTickMarkShape(),
      rangeThumbShape: const RoundRangeSliderThumbShape(),
      rangeTrackShape: const RoundedRectRangeSliderTrackShape(),
      rangeValueIndicatorShape: const PaddleRangeSliderValueIndicatorShape(),
      valueIndicatorTextStyle: valueIndicatorTextStyle,
      showValueIndicator: ShowValueIndicator.onlyForDiscrete,
    );
  }

  /// The height of the [Slider] track.
  final double? trackHeight;

  /// The color of the [Slider] track between the [Slider.min] position and the
  /// current thumb position.
  final Color? activeTrackColor;

  /// The color of the [Slider] track between the current thumb position and the
  /// [Slider.max] position.
  final Color? inactiveTrackColor;

  /// The color of the [Slider] track between the current thumb position and the
  /// [Slider.secondaryTrackValue] position.
  final Color? secondaryActiveTrackColor;

  /// The color of the [Slider] track between the [Slider.min] position and the
  /// current thumb position when the [Slider] is disabled.
  final Color? disabledActiveTrackColor;

  /// The color of the [Slider] track between the current thumb position and the
  /// [Slider.secondaryTrackValue] position when the [Slider] is disabled.
  final Color? disabledSecondaryActiveTrackColor;

  /// The color of the [Slider] track between the current thumb position and the
  /// [Slider.max] position when the [Slider] is disabled.
  final Color? disabledInactiveTrackColor;

  /// The color of the track's tick marks that are drawn between the [Slider.min]
  /// position and the current thumb position.
  final Color? activeTickMarkColor;

  /// The color of the track's tick marks that are drawn between the current
  /// thumb position and the [Slider.max] position.
  final Color? inactiveTickMarkColor;

  /// The color of the track's tick marks that are drawn between the [Slider.min]
  /// position and the current thumb position when the [Slider] is disabled.
  final Color? disabledActiveTickMarkColor;

  /// The color of the track's tick marks that are drawn between the current
  /// thumb position and the [Slider.max] position when the [Slider] is
  /// disabled.
  final Color? disabledInactiveTickMarkColor;

  /// The color given to the [thumbShape] to draw itself with.
  final Color? thumbColor;

  /// The color given to the perimeter of the top [rangeThumbShape] when the
  /// thumbs are overlapping and the top [rangeValueIndicatorShape] when the
  /// value indicators are overlapping.
  final Color? overlappingShapeStrokeColor;

  /// The color given to the [thumbShape] to draw itself with when the
  /// [Slider] is disabled.
  final Color? disabledThumbColor;

  /// The color of the overlay drawn around the slider thumb when it is
  /// pressed, focused, or hovered.
  ///
  /// This is typically a semi-transparent color.
  final Color? overlayColor;

  /// The color given to the [valueIndicatorShape] to draw itself with.
  final Color? valueIndicatorColor;

  /// The color given to the [valueIndicatorShape] stroke.
  final Color? valueIndicatorStrokeColor;

  /// The shape that will be used to draw the [Slider]'s overlay.
  ///
  /// Both the [overlayColor] and a non default [overlayShape] may be specified.
  /// The default [overlayShape] refers to the [overlayColor].
  ///
  /// The default value is [RoundSliderOverlayShape].
  final SliderComponentShape? overlayShape;

  /// The shape that will be used to draw the [Slider]'s tick marks.
  ///
  /// The [SliderTickMarkShape.getPreferredSize] is used to help determine the
  /// location of each tick mark on the track. The slider's minimum size will
  /// be at least this big.
  ///
  /// The default value is [RoundSliderTickMarkShape].
  ///
  /// See also:
  ///
  ///  * [RoundRangeSliderTickMarkShape], which is the default tick mark
  ///    shape for the range slider.
  final SliderTickMarkShape? tickMarkShape;

  /// The shape that will be used to draw the [Slider]'s thumb.
  ///
  /// The default value is [RoundSliderThumbShape].
  ///
  /// See also:
  ///
  ///  * [RoundRangeSliderThumbShape], which is the default thumb shape for
  ///    the [RangeSlider].
  final SliderComponentShape? thumbShape;

  /// The shape that will be used to draw the [Slider]'s track.
  ///
  /// The [SliderTrackShape.getPreferredRect] method is used to map
  /// slider-relative gesture coordinates to the correct thumb position on the
  /// track. It is also used to horizontally position tick marks, when the
  /// slider is discrete.
  ///
  /// The default value is [RoundedRectSliderTrackShape].
  ///
  /// See also:
  ///
  ///  * [RoundedRectRangeSliderTrackShape], which is the default track
  ///    shape for the [RangeSlider].
  final SliderTrackShape? trackShape;

  /// The shape that will be used to draw the [Slider]'s value
  /// indicator.
  ///
  /// The default value is [PaddleSliderValueIndicatorShape].
  ///
  /// See also:
  ///
  ///  * [PaddleRangeSliderValueIndicatorShape], which is the default value
  ///    indicator shape for the [RangeSlider].
  final SliderComponentShape? valueIndicatorShape;

  /// The shape that will be used to draw the [RangeSlider]'s tick marks.
  ///
  /// The [RangeSliderTickMarkShape.getPreferredSize] is used to help determine
  /// the location of each tick mark on the track. The slider's minimum size
  /// will be at least this big.
  ///
  /// The default value is [RoundRangeSliderTickMarkShape].
  ///
  /// See also:
  ///
  ///  * [RoundSliderTickMarkShape], which is the default tick mark shape
  ///    for the [Slider].
  final RangeSliderTickMarkShape? rangeTickMarkShape;

  /// The shape that will be used for the [RangeSlider]'s thumbs.
  ///
  /// By default the same shape is used for both thumbs, but strokes the top
  /// thumb when it overlaps the bottom thumb. The top thumb is always the last
  /// selected thumb.
  ///
  /// The default value is [RoundRangeSliderThumbShape].
  ///
  /// See also:
  ///
  ///  * [RoundSliderThumbShape], which is the default thumb shape for the
  ///    [Slider].
  final RangeSliderThumbShape? rangeThumbShape;

  /// The shape that will be used to draw the [RangeSlider]'s track.
  ///
  /// The [SliderTrackShape.getPreferredRect] method is used to map
  /// slider-relative gesture coordinates to the correct thumb position on the
  /// track. It is also used to horizontally position the tick marks, when the
  /// slider is discrete.
  ///
  /// The default value is [RoundedRectRangeSliderTrackShape].
  ///
  /// See also:
  ///
  ///  * [RoundedRectSliderTrackShape], which is the default track
  ///    shape for the [Slider].
  final RangeSliderTrackShape? rangeTrackShape;

  /// The shape that will be used for the [RangeSlider]'s value indicators.
  ///
  /// The default shape uses the same value indicator for each thumb, but
  /// strokes the top value indicator when it overlaps the bottom value
  /// indicator. The top indicator corresponds to the top thumb, which is always
  /// the most recently selected thumb.
  ///
  /// The default value is [PaddleRangeSliderValueIndicatorShape].
  ///
  /// See also:
  ///
  ///  * [PaddleSliderValueIndicatorShape], which is the default value
  ///    indicator shape for the [Slider].
  final RangeSliderValueIndicatorShape? rangeValueIndicatorShape;

  /// Whether the value indicator should be shown for different types of
  /// sliders.
  ///
  /// By default, [showValueIndicator] is set to
  /// [ShowValueIndicator.onlyForDiscrete]. The value indicator is only shown
  /// when the thumb is being touched.
  final ShowValueIndicator? showValueIndicator;

  /// The text style for the text on the value indicator.
  final TextStyle? valueIndicatorTextStyle;

  /// Limits the thumb's separation distance.
  ///
  /// Use this only if you want to control the visual appearance of the thumbs
  /// in terms of a logical pixel value. This can be done when you want a
  /// specific look for thumbs when they are close together. To limit with the
  /// real values, rather than logical pixels, the values can be restricted by
  /// the parent.
  final double? minThumbSeparation;

  /// Determines which thumb should be selected when the slider is interacted
  /// with.
  ///
  /// If null, the default thumb selector finds the closest thumb, excluding
  /// taps that are between the thumbs and not within any one touch target.
  /// When the selection is within the touch target bounds of both thumbs, no
  /// thumb is selected until the selection is moved.
  ///
  /// Override this for custom thumb selection.
  final RangeThumbSelector? thumbSelector;

  /// {@macro flutter.material.slider.mouseCursor}
  ///
  /// If specified, overrides the default value of [Slider.mouseCursor].
  final MaterialStateProperty<MouseCursor?>? mouseCursor;

  /// Allowed way for the user to interact with the [Slider].
  ///
  /// If specified, overrides the default value of [Slider.allowedInteraction].
  final SliderInteraction? allowedInteraction;

  /// Determines the padding around the [Slider].
  ///
  /// If specified, this padding overrides the default vertical padding of
  /// the [Slider], defaults to the height of the overlay shape, and the
  /// horizontal padding, defaults to the width of the thumb shape or
  /// overlay shape, whichever is larger.
  final EdgeInsetsGeometry? padding;

  /// The size of the [HandleThumbShape] thumb.
  ///
  /// If [SliderThemeData.thumbShape] is [HandleThumbShape], this property is used to
  /// set the size of the thumb. Otherwise, the default thumb size is 4 pixels for the
  /// width and 44 pixels for the height.
  final MaterialStateProperty<Size?>? thumbSize;

  /// The size of the gap between the active and inactive tracks of the [GappedSliderTrackShape].
  ///
  /// If [SliderThemeData.trackShape] is [GappedSliderTrackShape], this property
  /// is used to set the gap between the active and inactive tracks. Otherwise,
  /// the default gap size is 6.0 pixels.
  ///
  /// The Slider defaults to [GappedSliderTrackShape] when the track shape is
  /// not specified, and the [trackGap] can be used to adjust the gap size.
  ///
  /// If [Slider.year2023] is false or [ThemeData.useMaterial3] is false, then
  /// the Slider track shape defaults to [RoundedRectSliderTrackShape] and the
  /// [trackGap] is ignored. In this case, set the track shape to
  /// [GappedSliderTrackShape] to use the [trackGap].
  ///
  /// Defaults to 6.0 pixels of gap between the active and inactive tracks.
  final double? trackGap;

  /// Overrides the default value of [Slider.year2023].
  ///
  /// When true, the [Slider] will use the 2023 Material Design 3 appearance.
  /// Defaults to true.
  ///
  /// If this is set to false, the [Slider] will use the latest Material Design 3
  /// appearance, which was introduced in December 2023.
  ///
  /// If [ThemeData.useMaterial3] is false, then this property is ignored.
  @Deprecated(
    'Set this flag to false to opt into the 2024 slider appearance. Defaults to true. '
    'In the future, this flag will default to false. Use SliderThemeData to customize individual properties. '
    'This feature was deprecated after v3.27.0-0.2.pre.',
  )
  final bool? year2023;

  /// Creates a copy of this object but with the given fields replaced with the
  /// new values.
  SliderThemeData copyWith({
    double? trackHeight,
    Color? activeTrackColor,
    Color? inactiveTrackColor,
    Color? secondaryActiveTrackColor,
    Color? disabledActiveTrackColor,
    Color? disabledInactiveTrackColor,
    Color? disabledSecondaryActiveTrackColor,
    Color? activeTickMarkColor,
    Color? inactiveTickMarkColor,
    Color? disabledActiveTickMarkColor,
    Color? disabledInactiveTickMarkColor,
    Color? thumbColor,
    Color? overlappingShapeStrokeColor,
    Color? disabledThumbColor,
    Color? overlayColor,
    Color? valueIndicatorColor,
    Color? valueIndicatorStrokeColor,
    SliderComponentShape? overlayShape,
    SliderTickMarkShape? tickMarkShape,
    SliderComponentShape? thumbShape,
    SliderTrackShape? trackShape,
    SliderComponentShape? valueIndicatorShape,
    RangeSliderTickMarkShape? rangeTickMarkShape,
    RangeSliderThumbShape? rangeThumbShape,
    RangeSliderTrackShape? rangeTrackShape,
    RangeSliderValueIndicatorShape? rangeValueIndicatorShape,
    ShowValueIndicator? showValueIndicator,
    TextStyle? valueIndicatorTextStyle,
    double? minThumbSeparation,
    RangeThumbSelector? thumbSelector,
    MaterialStateProperty<MouseCursor?>? mouseCursor,
    SliderInteraction? allowedInteraction,
    EdgeInsetsGeometry? padding,
    MaterialStateProperty<Size?>? thumbSize,
    double? trackGap,
    bool? year2023,
  }) {
    return SliderThemeData(
      trackHeight: trackHeight ?? this.trackHeight,
      activeTrackColor: activeTrackColor ?? this.activeTrackColor,
      inactiveTrackColor: inactiveTrackColor ?? this.inactiveTrackColor,
      secondaryActiveTrackColor: secondaryActiveTrackColor ?? this.secondaryActiveTrackColor,
      disabledActiveTrackColor: disabledActiveTrackColor ?? this.disabledActiveTrackColor,
      disabledInactiveTrackColor: disabledInactiveTrackColor ?? this.disabledInactiveTrackColor,
      disabledSecondaryActiveTrackColor:
          disabledSecondaryActiveTrackColor ?? this.disabledSecondaryActiveTrackColor,
      activeTickMarkColor: activeTickMarkColor ?? this.activeTickMarkColor,
      inactiveTickMarkColor: inactiveTickMarkColor ?? this.inactiveTickMarkColor,
      disabledActiveTickMarkColor: disabledActiveTickMarkColor ?? this.disabledActiveTickMarkColor,
      disabledInactiveTickMarkColor:
          disabledInactiveTickMarkColor ?? this.disabledInactiveTickMarkColor,
      thumbColor: thumbColor ?? this.thumbColor,
      overlappingShapeStrokeColor: overlappingShapeStrokeColor ?? this.overlappingShapeStrokeColor,
      disabledThumbColor: disabledThumbColor ?? this.disabledThumbColor,
      overlayColor: overlayColor ?? this.overlayColor,
      valueIndicatorColor: valueIndicatorColor ?? this.valueIndicatorColor,
      valueIndicatorStrokeColor: valueIndicatorStrokeColor ?? this.valueIndicatorStrokeColor,
      overlayShape: overlayShape ?? this.overlayShape,
      tickMarkShape: tickMarkShape ?? this.tickMarkShape,
      thumbShape: thumbShape ?? this.thumbShape,
      trackShape: trackShape ?? this.trackShape,
      valueIndicatorShape: valueIndicatorShape ?? this.valueIndicatorShape,
      rangeTickMarkShape: rangeTickMarkShape ?? this.rangeTickMarkShape,
      rangeThumbShape: rangeThumbShape ?? this.rangeThumbShape,
      rangeTrackShape: rangeTrackShape ?? this.rangeTrackShape,
      rangeValueIndicatorShape: rangeValueIndicatorShape ?? this.rangeValueIndicatorShape,
      showValueIndicator: showValueIndicator ?? this.showValueIndicator,
      valueIndicatorTextStyle: valueIndicatorTextStyle ?? this.valueIndicatorTextStyle,
      minThumbSeparation: minThumbSeparation ?? this.minThumbSeparation,
      thumbSelector: thumbSelector ?? this.thumbSelector,
      mouseCursor: mouseCursor ?? this.mouseCursor,
      allowedInteraction: allowedInteraction ?? this.allowedInteraction,
      padding: padding ?? this.padding,
      thumbSize: thumbSize ?? this.thumbSize,
      trackGap: trackGap ?? this.trackGap,
      year2023: year2023 ?? this.year2023,
    );
  }

  /// Linearly interpolate between two slider themes.
  ///
  /// {@macro dart.ui.shadow.lerp}
  static SliderThemeData lerp(SliderThemeData a, SliderThemeData b, double t) {
    if (identical(a, b)) {
      return a;
    }
    return SliderThemeData(
      trackHeight: lerpDouble(a.trackHeight, b.trackHeight, t),
      activeTrackColor: Color.lerp(a.activeTrackColor, b.activeTrackColor, t),
      inactiveTrackColor: Color.lerp(a.inactiveTrackColor, b.inactiveTrackColor, t),
      secondaryActiveTrackColor: Color.lerp(
        a.secondaryActiveTrackColor,
        b.secondaryActiveTrackColor,
        t,
      ),
      disabledActiveTrackColor: Color.lerp(
        a.disabledActiveTrackColor,
        b.disabledActiveTrackColor,
        t,
      ),
      disabledInactiveTrackColor: Color.lerp(
        a.disabledInactiveTrackColor,
        b.disabledInactiveTrackColor,
        t,
      ),
      disabledSecondaryActiveTrackColor: Color.lerp(
        a.disabledSecondaryActiveTrackColor,
        b.disabledSecondaryActiveTrackColor,
        t,
      ),
      activeTickMarkColor: Color.lerp(a.activeTickMarkColor, b.activeTickMarkColor, t),
      inactiveTickMarkColor: Color.lerp(a.inactiveTickMarkColor, b.inactiveTickMarkColor, t),
      disabledActiveTickMarkColor: Color.lerp(
        a.disabledActiveTickMarkColor,
        b.disabledActiveTickMarkColor,
        t,
      ),
      disabledInactiveTickMarkColor: Color.lerp(
        a.disabledInactiveTickMarkColor,
        b.disabledInactiveTickMarkColor,
        t,
      ),
      thumbColor: Color.lerp(a.thumbColor, b.thumbColor, t),
      overlappingShapeStrokeColor: Color.lerp(
        a.overlappingShapeStrokeColor,
        b.overlappingShapeStrokeColor,
        t,
      ),
      disabledThumbColor: Color.lerp(a.disabledThumbColor, b.disabledThumbColor, t),
      overlayColor: Color.lerp(a.overlayColor, b.overlayColor, t),
      valueIndicatorColor: Color.lerp(a.valueIndicatorColor, b.valueIndicatorColor, t),
      valueIndicatorStrokeColor: Color.lerp(
        a.valueIndicatorStrokeColor,
        b.valueIndicatorStrokeColor,
        t,
      ),
      overlayShape: t < 0.5 ? a.overlayShape : b.overlayShape,
      tickMarkShape: t < 0.5 ? a.tickMarkShape : b.tickMarkShape,
      thumbShape: t < 0.5 ? a.thumbShape : b.thumbShape,
      trackShape: t < 0.5 ? a.trackShape : b.trackShape,
      valueIndicatorShape: t < 0.5 ? a.valueIndicatorShape : b.valueIndicatorShape,
      rangeTickMarkShape: t < 0.5 ? a.rangeTickMarkShape : b.rangeTickMarkShape,
      rangeThumbShape: t < 0.5 ? a.rangeThumbShape : b.rangeThumbShape,
      rangeTrackShape: t < 0.5 ? a.rangeTrackShape : b.rangeTrackShape,
      rangeValueIndicatorShape: t < 0.5 ? a.rangeValueIndicatorShape : b.rangeValueIndicatorShape,
      showValueIndicator: t < 0.5 ? a.showValueIndicator : b.showValueIndicator,
      valueIndicatorTextStyle: TextStyle.lerp(
        a.valueIndicatorTextStyle,
        b.valueIndicatorTextStyle,
        t,
      ),
      minThumbSeparation: lerpDouble(a.minThumbSeparation, b.minThumbSeparation, t),
      thumbSelector: t < 0.5 ? a.thumbSelector : b.thumbSelector,
      mouseCursor: t < 0.5 ? a.mouseCursor : b.mouseCursor,
      allowedInteraction: t < 0.5 ? a.allowedInteraction : b.allowedInteraction,
      padding: EdgeInsetsGeometry.lerp(a.padding, b.padding, t),
      thumbSize: MaterialStateProperty.lerp<Size?>(a.thumbSize, b.thumbSize, t, Size.lerp),
      trackGap: lerpDouble(a.trackGap, b.trackGap, t),
      year2023: t < 0.5 ? a.year2023 : b.year2023,
    );
  }

  @override
  int get hashCode => Object.hash(
    trackHeight,
    activeTrackColor,
    inactiveTrackColor,
    secondaryActiveTrackColor,
    disabledActiveTrackColor,
    disabledInactiveTrackColor,
    disabledSecondaryActiveTrackColor,
    activeTickMarkColor,
    inactiveTickMarkColor,
    disabledActiveTickMarkColor,
    disabledInactiveTickMarkColor,
    thumbColor,
    overlappingShapeStrokeColor,
    disabledThumbColor,
    overlayColor,
    valueIndicatorColor,
    overlayShape,
    tickMarkShape,
    thumbShape,
    Object.hash(
      trackShape,
      valueIndicatorShape,
      rangeTickMarkShape,
      rangeThumbShape,
      rangeTrackShape,
      rangeValueIndicatorShape,
      showValueIndicator,
      valueIndicatorTextStyle,
      minThumbSeparation,
      thumbSelector,
      mouseCursor,
      allowedInteraction,
      padding,
      thumbSize,
      trackGap,
      year2023,
    ),
  );

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is SliderThemeData &&
        other.trackHeight == trackHeight &&
        other.activeTrackColor == activeTrackColor &&
        other.inactiveTrackColor == inactiveTrackColor &&
        other.secondaryActiveTrackColor == secondaryActiveTrackColor &&
        other.disabledActiveTrackColor == disabledActiveTrackColor &&
        other.disabledInactiveTrackColor == disabledInactiveTrackColor &&
        other.disabledSecondaryActiveTrackColor == disabledSecondaryActiveTrackColor &&
        other.activeTickMarkColor == activeTickMarkColor &&
        other.inactiveTickMarkColor == inactiveTickMarkColor &&
        other.disabledActiveTickMarkColor == disabledActiveTickMarkColor &&
        other.disabledInactiveTickMarkColor == disabledInactiveTickMarkColor &&
        other.thumbColor == thumbColor &&
        other.overlappingShapeStrokeColor == overlappingShapeStrokeColor &&
        other.disabledThumbColor == disabledThumbColor &&
        other.overlayColor == overlayColor &&
        other.valueIndicatorColor == valueIndicatorColor &&
        other.valueIndicatorStrokeColor == valueIndicatorStrokeColor &&
        other.overlayShape == overlayShape &&
        other.tickMarkShape == tickMarkShape &&
        other.thumbShape == thumbShape &&
        other.trackShape == trackShape &&
        other.valueIndicatorShape == valueIndicatorShape &&
        other.rangeTickMarkShape == rangeTickMarkShape &&
        other.rangeThumbShape == rangeThumbShape &&
        other.rangeTrackShape == rangeTrackShape &&
        other.rangeValueIndicatorShape == rangeValueIndicatorShape &&
        other.showValueIndicator == showValueIndicator &&
        other.valueIndicatorTextStyle == valueIndicatorTextStyle &&
        other.minThumbSeparation == minThumbSeparation &&
        other.thumbSelector == thumbSelector &&
        other.mouseCursor == mouseCursor &&
        other.allowedInteraction == allowedInteraction &&
        other.padding == padding &&
        other.thumbSize == thumbSize &&
        other.trackGap == trackGap &&
        other.year2023 == year2023;
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    const SliderThemeData defaultData = SliderThemeData();
    properties.add(
      DoubleProperty('trackHeight', trackHeight, defaultValue: defaultData.trackHeight),
    );
    properties.add(
      ColorProperty(
        'activeTrackColor',
        activeTrackColor,
        defaultValue: defaultData.activeTrackColor,
      ),
    );
    properties.add(
      ColorProperty(
        'inactiveTrackColor',
        inactiveTrackColor,
        defaultValue: defaultData.inactiveTrackColor,
      ),
    );
    properties.add(
      ColorProperty(
        'secondaryActiveTrackColor',
        secondaryActiveTrackColor,
        defaultValue: defaultData.secondaryActiveTrackColor,
      ),
    );
    properties.add(
      ColorProperty(
        'disabledActiveTrackColor',
        disabledActiveTrackColor,
        defaultValue: defaultData.disabledActiveTrackColor,
      ),
    );
    properties.add(
      ColorProperty(
        'disabledInactiveTrackColor',
        disabledInactiveTrackColor,
        defaultValue: defaultData.disabledInactiveTrackColor,
      ),
    );
    properties.add(
      ColorProperty(
        'disabledSecondaryActiveTrackColor',
        disabledSecondaryActiveTrackColor,
        defaultValue: defaultData.disabledSecondaryActiveTrackColor,
      ),
    );
    properties.add(
      ColorProperty(
        'activeTickMarkColor',
        activeTickMarkColor,
        defaultValue: defaultData.activeTickMarkColor,
      ),
    );
    properties.add(
      ColorProperty(
        'inactiveTickMarkColor',
        inactiveTickMarkColor,
        defaultValue: defaultData.inactiveTickMarkColor,
      ),
    );
    properties.add(
      ColorProperty(
        'disabledActiveTickMarkColor',
        disabledActiveTickMarkColor,
        defaultValue: defaultData.disabledActiveTickMarkColor,
      ),
    );
    properties.add(
      ColorProperty(
        'disabledInactiveTickMarkColor',
        disabledInactiveTickMarkColor,
        defaultValue: defaultData.disabledInactiveTickMarkColor,
      ),
    );
    properties.add(ColorProperty('thumbColor', thumbColor, defaultValue: defaultData.thumbColor));
    properties.add(
      ColorProperty(
        'overlappingShapeStrokeColor',
        overlappingShapeStrokeColor,
        defaultValue: defaultData.overlappingShapeStrokeColor,
      ),
    );
    properties.add(
      ColorProperty(
        'disabledThumbColor',
        disabledThumbColor,
        defaultValue: defaultData.disabledThumbColor,
      ),
    );
    properties.add(
      ColorProperty('overlayColor', overlayColor, defaultValue: defaultData.overlayColor),
    );
    properties.add(
      ColorProperty(
        'valueIndicatorColor',
        valueIndicatorColor,
        defaultValue: defaultData.valueIndicatorColor,
      ),
    );
    properties.add(
      ColorProperty(
        'valueIndicatorStrokeColor',
        valueIndicatorStrokeColor,
        defaultValue: defaultData.valueIndicatorStrokeColor,
      ),
    );
    properties.add(
      DiagnosticsProperty<SliderComponentShape>(
        'overlayShape',
        overlayShape,
        defaultValue: defaultData.overlayShape,
      ),
    );
    properties.add(
      DiagnosticsProperty<SliderTickMarkShape>(
        'tickMarkShape',
        tickMarkShape,
        defaultValue: defaultData.tickMarkShape,
      ),
    );
    properties.add(
      DiagnosticsProperty<SliderComponentShape>(
        'thumbShape',
        thumbShape,
        defaultValue: defaultData.thumbShape,
      ),
    );
    properties.add(
      DiagnosticsProperty<SliderTrackShape>(
        'trackShape',
        trackShape,
        defaultValue: defaultData.trackShape,
      ),
    );
    properties.add(
      DiagnosticsProperty<SliderComponentShape>(
        'valueIndicatorShape',
        valueIndicatorShape,
        defaultValue: defaultData.valueIndicatorShape,
      ),
    );
    properties.add(
      DiagnosticsProperty<RangeSliderTickMarkShape>(
        'rangeTickMarkShape',
        rangeTickMarkShape,
        defaultValue: defaultData.rangeTickMarkShape,
      ),
    );
    properties.add(
      DiagnosticsProperty<RangeSliderThumbShape>(
        'rangeThumbShape',
        rangeThumbShape,
        defaultValue: defaultData.rangeThumbShape,
      ),
    );
    properties.add(
      DiagnosticsProperty<RangeSliderTrackShape>(
        'rangeTrackShape',
        rangeTrackShape,
        defaultValue: defaultData.rangeTrackShape,
      ),
    );
    properties.add(
      DiagnosticsProperty<RangeSliderValueIndicatorShape>(
        'rangeValueIndicatorShape',
        rangeValueIndicatorShape,
        defaultValue: defaultData.rangeValueIndicatorShape,
      ),
    );
    properties.add(
      EnumProperty<ShowValueIndicator>(
        'showValueIndicator',
        showValueIndicator,
        defaultValue: defaultData.showValueIndicator,
      ),
    );
    properties.add(
      DiagnosticsProperty<TextStyle>(
        'valueIndicatorTextStyle',
        valueIndicatorTextStyle,
        defaultValue: defaultData.valueIndicatorTextStyle,
      ),
    );
    properties.add(
      DoubleProperty(
        'minThumbSeparation',
        minThumbSeparation,
        defaultValue: defaultData.minThumbSeparation,
      ),
    );
    properties.add(
      DiagnosticsProperty<RangeThumbSelector>(
        'thumbSelector',
        thumbSelector,
        defaultValue: defaultData.thumbSelector,
      ),
    );
    properties.add(
      DiagnosticsProperty<MaterialStateProperty<MouseCursor?>>(
        'mouseCursor',
        mouseCursor,
        defaultValue: defaultData.mouseCursor,
      ),
    );
    properties.add(
      EnumProperty<SliderInteraction>(
        'allowedInteraction',
        allowedInteraction,
        defaultValue: defaultData.allowedInteraction,
      ),
    );
    properties.add(
      DiagnosticsProperty<EdgeInsetsGeometry>(
        'padding',
        padding,
        defaultValue: defaultData.padding,
      ),
    );
    properties.add(
      DiagnosticsProperty<MaterialStateProperty<Size?>>(
        'thumbSize',
        thumbSize,
        defaultValue: defaultData.thumbSize,
      ),
    );
    properties.add(DoubleProperty('trackGap', trackGap, defaultValue: defaultData.trackGap));
    properties.add(
      DiagnosticsProperty<bool>('year2023', year2023, defaultValue: defaultData.year2023),
    );
  }
}
