// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'range_slider.dart';
library;

import 'dart:math' as math;
import 'dart:ui' show Path, lerpDouble;

import 'package:flutter/foundation.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/widgets.dart';

import 'colors.dart';
import 'material_state.dart';
import 'slider.dart';
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
      'Use SliderTheme to customize the Slider appearance. '
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
    'Use SliderTheme to customize the Slider appearance. '
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

/// Base class for slider thumb, thumb overlay, and value indicator shapes.
///
/// Create a subclass of this if you would like a custom shape.
///
/// All shapes are painted to the same canvas and ordering is important.
/// The overlay is painted first, then the value indicator, then the thumb.
///
/// The thumb painting can be skipped by specifying [noThumb] for
/// [SliderThemeData.thumbShape].
///
/// The overlay painting can be skipped by specifying [noOverlay] for
/// [SliderThemeData.overlayShape].
///
/// See also:
///
///  * [RoundSliderThumbShape], which is the default [Slider]'s thumb shape that
///    paints a solid circle.
///  * [RoundSliderOverlayShape], which is the default [Slider] and
///    [RangeSlider]'s overlay shape that paints a transparent circle.
///  * [PaddleSliderValueIndicatorShape], which is the default [Slider]'s value
///    indicator shape that paints a custom path with text in it.
abstract class SliderComponentShape {
  /// This abstract const constructor enables subclasses to provide
  /// const constructors so that they can be used in const expressions.
  const SliderComponentShape();

  /// Returns the preferred size of the shape, based on the given conditions.
  Size getPreferredSize(bool isEnabled, bool isDiscrete);

  /// Paints the shape, taking into account the state passed to it.
  ///
  /// {@template flutter.material.SliderComponentShape.paint.context}
  /// The `context` argument is the same as the one that includes the [Slider]'s
  /// render box.
  /// {@endtemplate}
  ///
  /// {@template flutter.material.SliderComponentShape.paint.center}
  /// The `center` argument is the offset for where this shape's center should be
  /// painted. This offset is relative to the origin of the [context] canvas.
  /// {@endtemplate}
  ///
  /// The `activationAnimation` argument is an animation triggered when the user
  /// begins to interact with the slider. It reverses when the user stops interacting
  /// with the slider.
  ///
  /// {@template flutter.material.SliderComponentShape.paint.enableAnimation}
  /// The `enableAnimation` argument is an animation triggered when the [Slider]
  /// is enabled, and it reverses when the slider is disabled. The [Slider] is
  /// enabled when [Slider.onChanged] is not null.Use this to paint intermediate
  /// frames for this shape when the slider changes enabled state.
  /// {@endtemplate}
  ///
  /// {@template flutter.material.SliderComponentShape.paint.isDiscrete}
  /// The `isDiscrete` argument is true if [Slider.divisions] is non-null. When
  /// true, the slider will render tick marks on top of the track.
  /// {@endtemplate}
  ///
  /// If the `labelPainter` argument is non-null, then [TextPainter.paint]
  /// should be called on the `labelPainter` with the location that the label
  /// should appear. If the `labelPainter` argument is null, then no label was
  /// supplied to the [Slider].
  ///
  /// {@template flutter.material.SliderComponentShape.paint.parentBox}
  /// The `parentBox` argument is the [RenderBox] of the [Slider]. Its attributes,
  /// such as size, can be used to assist in painting this shape.
  /// {@endtemplate}
  ///
  /// {@template flutter.material.SliderComponentShape.paint.sliderTheme}
  /// the `sliderTheme` argument is the theme assigned to the [Slider] that this
  /// shape belongs to.
  /// {@endtemplate}
  ///
  /// The `textDirection` argument can be used to determine how any extra text
  /// or graphics (besides the text painted by the `labelPainter`) should be
  /// positioned. The `labelPainter` already has the [textDirection] set.
  ///
  /// The `value` argument is the current parametric value (from 0.0 to 1.0) of
  /// the slider.
  ///
  /// {@template flutter.material.SliderComponentShape.paint.textScaleFactor}
  /// The `textScaleFactor` argument can be used to determine whether the
  /// component should paint larger or smaller, depending on whether
  /// [textScaleFactor] is greater than 1 for larger, and between 0 and 1 for
  /// smaller. It's usually computed from [MediaQueryData.textScaler].
  /// {@endtemplate}
  ///
  /// {@template flutter.material.SliderComponentShape.paint.sizeWithOverflow}
  /// The `sizeWithOverflow` argument can be used to determine the bounds the
  /// drawing of the components that are outside of the regular slider bounds.
  /// It's the size of the box, whose center is aligned with the slider's
  /// bounds, that the value indicators must be drawn within. Typically, it is
  /// bigger than the slider.
  /// {@endtemplate}
  void paint(
    PaintingContext context,
    Offset center, {
    required Animation<double> activationAnimation,
    required Animation<double> enableAnimation,
    required bool isDiscrete,
    required TextPainter labelPainter,
    required RenderBox parentBox,
    required SliderThemeData sliderTheme,
    required TextDirection textDirection,
    required double value,
    required double textScaleFactor,
    required Size sizeWithOverflow,
  });

  /// Special instance of [SliderComponentShape] to skip the thumb drawing.
  ///
  /// See also:
  ///
  ///  * [SliderThemeData.thumbShape], which is the shape that the [Slider]
  ///    uses when painting the thumb.
  static final SliderComponentShape noThumb = _EmptySliderComponentShape();

  /// Special instance of [SliderComponentShape] to skip the overlay drawing.
  ///
  /// See also:
  ///
  ///  * [SliderThemeData.overlayShape], which is the shape that the [Slider]
  ///    uses when painting the overlay.
  static final SliderComponentShape noOverlay = _EmptySliderComponentShape();
}

/// Base class for [Slider] tick mark shapes.
///
/// Create a subclass of this if you would like a custom slider tick mark shape.
///
/// The tick mark painting can be skipped by specifying [noTickMark] for
/// [SliderThemeData.tickMarkShape].
///
/// See also:
///
///  * [RoundSliderTickMarkShape], which is the default [Slider]'s tick mark
///    shape that paints a solid circle.
///  * [SliderTrackShape], which can be used to create custom shapes for the
///    [Slider]'s track.
///  * [SliderComponentShape], which can be used to create custom shapes for
///    the [Slider]'s thumb, overlay, and value indicator and the
///    [RangeSlider]'s overlay.
abstract class SliderTickMarkShape {
  /// This abstract const constructor enables subclasses to provide
  /// const constructors so that they can be used in const expressions.
  const SliderTickMarkShape();

  /// Returns the preferred size of the shape.
  ///
  /// It is used to help position the tick marks within the slider.
  ///
  /// {@macro flutter.material.SliderComponentShape.paint.sliderTheme}
  ///
  /// {@template flutter.material.SliderTickMarkShape.getPreferredSize.isEnabled}
  /// The `isEnabled` argument is false when [Slider.onChanged] is null and true
  /// otherwise. When true, the slider will respond to input.
  /// {@endtemplate}
  Size getPreferredSize({required SliderThemeData sliderTheme, required bool isEnabled});

  /// Paints the slider track.
  ///
  /// {@macro flutter.material.SliderComponentShape.paint.context}
  ///
  /// {@macro flutter.material.SliderComponentShape.paint.center}
  ///
  /// {@macro flutter.material.SliderComponentShape.paint.parentBox}
  ///
  /// {@macro flutter.material.SliderComponentShape.paint.sliderTheme}
  ///
  /// {@macro flutter.material.SliderComponentShape.paint.enableAnimation}
  ///
  /// {@macro flutter.material.SliderTickMarkShape.getPreferredSize.isEnabled}
  ///
  /// The `textDirection` argument can be used to determine how the tick marks
  /// are painting depending on whether they are on an active track segment or
  /// not. The track segment between the start of the slider and the thumb is
  /// the active track segment. The track segment between the thumb and the end
  /// of the slider is the inactive track segment. In LTR text direction, the
  /// start of the slider is on the left, and in RTL text direction, the start
  /// of the slider is on the right.
  void paint(
    PaintingContext context,
    Offset center, {
    required RenderBox parentBox,
    required SliderThemeData sliderTheme,
    required Animation<double> enableAnimation,
    required Offset thumbCenter,
    required bool isEnabled,
    required TextDirection textDirection,
  });

  /// Special instance of [SliderTickMarkShape] to skip the tick mark painting.
  ///
  /// See also:
  ///
  ///  * [SliderThemeData.tickMarkShape], which is the shape that the [Slider]
  ///    uses when painting tick marks.
  static final SliderTickMarkShape noTickMark = _EmptySliderTickMarkShape();
}

/// Base class for slider track shapes.
///
/// The slider's thumb moves along the track. A discrete slider's tick marks
/// are drawn after the track, but before the thumb, and are aligned with the
/// track.
///
/// The [getPreferredRect] helps position the slider thumb and tick marks
/// relative to the track.
///
/// See also:
///
///  * [RoundedRectSliderTrackShape] for the default [Slider]'s track shape that
///    paints a stadium-like track.
///  * [SliderTickMarkShape], which can be used to create custom shapes for the
///    [Slider]'s tick marks.
///  * [SliderComponentShape], which can be used to create custom shapes for
///    the [Slider]'s thumb, overlay, and value indicator and the
///    [RangeSlider]'s overlay.
abstract class SliderTrackShape {
  /// This abstract const constructor enables subclasses to provide
  /// const constructors so that they can be used in const expressions.
  const SliderTrackShape();

  /// Returns the preferred bounds of the shape.
  ///
  /// It is used to provide horizontal boundaries for the thumb's position, and
  /// to help position the slider thumb and tick marks relative to the track.
  ///
  /// The `parentBox` argument can be used to help determine the preferredRect relative to
  /// attributes of the render box of the slider itself, such as size.
  ///
  /// The `offset` argument is relative to the caller's bounding box. It can be used to
  /// convert gesture coordinates from global to slider-relative coordinates.
  ///
  /// {@macro flutter.material.SliderComponentShape.paint.sliderTheme}
  ///
  /// {@macro flutter.material.SliderTickMarkShape.getPreferredSize.isEnabled}
  ///
  /// {@macro flutter.material.SliderComponentShape.paint.isDiscrete}
  Rect getPreferredRect({
    required RenderBox parentBox,
    Offset offset = Offset.zero,
    required SliderThemeData sliderTheme,
    bool isEnabled,
    bool isDiscrete,
  });

  /// Paints the track shape based on the state passed to it.
  ///
  /// {@macro flutter.material.SliderComponentShape.paint.context}
  ///
  /// The `offset` argument the offset of the origin of the `parentBox` to the
  /// origin of its `context` canvas. This shape must be painted relative to
  /// this offset. See [PaintingContextCallback].
  ///
  /// {@macro flutter.material.SliderComponentShape.paint.parentBox}
  ///
  /// {@macro flutter.material.SliderComponentShape.paint.sliderTheme}
  ///
  /// {@macro flutter.material.SliderComponentShape.paint.enableAnimation}
  ///
  /// The `thumbCenter` argument is the offset of the center of the thumb
  /// relative to the origin of the [PaintingContext.canvas]. It can be used as
  /// the point that divides the track into 2 segments.
  ///
  /// The `secondaryOffset` argument is the offset of the secondary value
  /// relative to the origin of the [PaintingContext.canvas].
  ///
  /// If not null, the track is divided into 3 segments.
  ///
  /// {@macro flutter.material.SliderTickMarkShape.getPreferredSize.isEnabled}
  ///
  /// {@macro flutter.material.SliderComponentShape.paint.isDiscrete}
  ///
  /// The `textDirection` argument can be used to determine how the track
  /// segments are painted depending on whether they are active or not.
  ///
  /// {@template flutter.material.SliderTrackShape.paint.trackSegment}
  /// The track segment between the start of the slider and the thumb is the
  /// active track segment. The track segment between the thumb and the end of the
  /// slider is the inactive track segment. In [TextDirection.ltr], the start of
  /// the slider is on the left, and in [TextDirection.rtl], the start of the
  /// slider is on the right.
  /// {@endtemplate}
  void paint(
    PaintingContext context,
    Offset offset, {
    required RenderBox parentBox,
    required SliderThemeData sliderTheme,
    required Animation<double> enableAnimation,
    required Offset thumbCenter,
    Offset? secondaryOffset,
    bool isEnabled,
    bool isDiscrete,
    required TextDirection textDirection,
  });

  /// Whether the track shape is rounded.
  ///
  /// This is used to determine the correct position of the thumb in relation to the track.
  bool get isRounded => false;
}

/// Base class for [RangeSlider] thumb shapes.
///
/// See also:
///
///  * [RoundRangeSliderThumbShape] for the default [RangeSlider]'s thumb shape
///    that paints a solid circle.
///  * [RangeSliderTickMarkShape], which can be used to create custom shapes for
///    the [RangeSlider]'s tick marks.
///  * [RangeSliderTrackShape], which can be used to create custom shapes for
///    the [RangeSlider]'s track.
///  * [RangeSliderValueIndicatorShape], which can be used to create custom
///    shapes for the [RangeSlider]'s value indicator.
///  * [SliderComponentShape], which can be used to create custom shapes for
///    the [Slider]'s thumb, overlay, and value indicator and the
///    [RangeSlider]'s overlay.
abstract class RangeSliderThumbShape {
  /// This abstract const constructor enables subclasses to provide
  /// const constructors so that they can be used in const expressions.
  const RangeSliderThumbShape();

  /// Returns the preferred size of the shape, based on the given conditions.
  ///
  /// {@template flutter.material.RangeSliderThumbShape.getPreferredSize.isDiscrete}
  /// The `isDiscrete` argument is true if [RangeSlider.divisions] is non-null.
  /// When true, the slider will render tick marks on top of the track.
  /// {@endtemplate}
  ///
  /// {@template flutter.material.RangeSliderThumbShape.getPreferredSize.isEnabled}
  /// The `isEnabled` argument is false when [RangeSlider.onChanged] is null and
  /// true otherwise. When true, the slider will respond to input.
  /// {@endtemplate}
  Size getPreferredSize(bool isEnabled, bool isDiscrete);

  /// Paints the thumb shape based on the state passed to it.
  ///
  /// {@template flutter.material.RangeSliderThumbShape.paint.context}
  /// The `context` argument represents the [RangeSlider]'s render box.
  /// {@endtemplate}
  ///
  /// {@macro flutter.material.SliderComponentShape.paint.center}
  ///
  /// {@template flutter.material.RangeSliderThumbShape.paint.activationAnimation}
  /// The `activationAnimation` argument is an animation triggered when the user
  /// begins to interact with the [RangeSlider]. It reverses when the user stops
  /// interacting with the slider.
  /// {@endtemplate}
  ///
  /// {@template flutter.material.RangeSliderThumbShape.paint.enableAnimation}
  /// The `enableAnimation` argument is an animation triggered when the
  /// [RangeSlider] is enabled, and it reverses when the slider is disabled. The
  /// [RangeSlider] is enabled when [RangeSlider.onChanged] is not null. Use
  /// this to paint intermediate frames for this shape when the slider changes
  /// enabled state.
  /// {@endtemplate}
  ///
  /// {@macro flutter.material.RangeSliderThumbShape.getPreferredSize.isDiscrete}
  ///
  /// {@macro flutter.material.RangeSliderThumbShape.getPreferredSize.isEnabled}
  ///
  /// If the `isOnTop` argument is true, this thumb is painted on top of the
  /// other slider thumb because this thumb is the one that was most recently
  /// selected.
  ///
  /// {@template flutter.material.RangeSliderThumbShape.paint.sliderTheme}
  /// The `sliderTheme` argument is the theme assigned to the [RangeSlider] that
  /// this shape belongs to.
  /// {@endtemplate}
  ///
  /// The `textDirection` argument can be used to determine how the orientation
  /// of either slider thumb should be changed, such as drawing different
  /// shapes for the left and right thumb.
  ///
  /// {@template flutter.material.RangeSliderThumbShape.paint.thumb}
  /// The `thumb` argument is the specifier for which of the two thumbs this
  /// method should paint (start or end).
  /// {@endtemplate}
  ///
  /// The `isPressed` argument can be used to give the selected thumb
  /// additional selected or pressed state visual feedback, such as a larger
  /// shadow.
  void paint(
    PaintingContext context,
    Offset center, {
    required Animation<double> activationAnimation,
    required Animation<double> enableAnimation,
    bool isDiscrete,
    bool isEnabled,
    bool isOnTop,
    TextDirection textDirection,
    required SliderThemeData sliderTheme,
    Thumb thumb,
    bool isPressed,
  });
}

/// Base class for [RangeSlider] value indicator shapes.
///
/// See also:
///
///  * [PaddleRangeSliderValueIndicatorShape] for the default [RangeSlider]'s
///    value indicator shape that paints a custom path with text in it.
///  * [RangeSliderTickMarkShape], which can be used to create custom shapes for
///    the [RangeSlider]'s tick marks.
///  * [RangeSliderThumbShape], which can be used to create custom shapes for
///    the [RangeSlider]'s thumb.
///  * [RangeSliderTrackShape], which can be used to create custom shapes for
///    the [RangeSlider]'s track.
///  * [SliderComponentShape], which can be used to create custom shapes for
///    the [Slider]'s thumb, overlay, and value indicator and the
///    [RangeSlider]'s overlay.
abstract class RangeSliderValueIndicatorShape {
  /// This abstract const constructor enables subclasses to provide
  /// const constructors so that they can be used in const expressions.
  const RangeSliderValueIndicatorShape();

  /// Returns the preferred size of the shape, based on the given conditions.
  ///
  /// {@macro flutter.material.RangeSliderThumbShape.getPreferredSize.isEnabled}
  ///
  /// {@macro flutter.material.RangeSliderThumbShape.getPreferredSize.isDiscrete}
  ///
  /// The `labelPainter` argument helps determine the width of the shape. It is
  /// variable width because it is derived from a formatted string.
  ///
  /// {@macro flutter.material.SliderComponentShape.paint.textScaleFactor}
  Size getPreferredSize(
    bool isEnabled,
    bool isDiscrete, {
    required TextPainter labelPainter,
    required double textScaleFactor,
  });

  /// Determines the best offset to keep this shape on the screen.
  ///
  /// Override this method when the center of the value indicator should be
  /// shifted from the vertical center of the thumb.
  double getHorizontalShift({
    RenderBox? parentBox,
    Offset? center,
    TextPainter? labelPainter,
    Animation<double>? activationAnimation,
    double? textScaleFactor,
    Size? sizeWithOverflow,
  }) {
    return 0;
  }

  /// Paints the value indicator shape based on the state passed to it.
  ///
  /// {@macro flutter.material.RangeSliderThumbShape.paint.context}
  ///
  /// {@macro flutter.material.SliderComponentShape.paint.center}
  ///
  /// {@macro flutter.material.RangeSliderThumbShape.paint.activationAnimation}
  ///
  /// {@macro flutter.material.RangeSliderThumbShape.paint.enableAnimation}
  ///
  /// {@macro flutter.material.RangeSliderThumbShape.getPreferredSize.isDiscrete}
  ///
  /// The `isOnTop` argument is the top-most value indicator between the two value
  /// indicators, which is always the indicator for the most recently selected thumb. In
  /// the default case, this is used to paint a stroke around the top indicator
  /// for better visibility between the two indicators.
  ///
  /// {@macro flutter.material.SliderComponentShape.paint.textScaleFactor}
  ///
  /// {@macro flutter.material.SliderComponentShape.paint.sizeWithOverflow}
  ///
  /// {@template flutter.material.RangeSliderValueIndicatorShape.paint.parentBox}
  /// The `parentBox` argument is the [RenderBox] of the [RangeSlider]. Its
  /// attributes, such as size, can be used to assist in painting this shape.
  /// {@endtemplate}
  ///
  /// {@macro flutter.material.RangeSliderThumbShape.paint.sliderTheme}
  ///
  /// The `textDirection` argument can be used to determine how any extra text
  /// or graphics, besides the text painted by the [labelPainter] should be
  /// positioned. The `labelPainter` argument already has the `textDirection`
  /// set.
  ///
  /// The `value` argument is the current parametric value (from 0.0 to 1.0) of
  /// the slider.
  ///
  /// {@macro flutter.material.RangeSliderThumbShape.paint.thumb}
  void paint(
    PaintingContext context,
    Offset center, {
    required Animation<double> activationAnimation,
    required Animation<double> enableAnimation,
    bool isDiscrete,
    bool isOnTop,
    required TextPainter labelPainter,
    double textScaleFactor,
    Size sizeWithOverflow,
    required RenderBox parentBox,
    required SliderThemeData sliderTheme,
    TextDirection textDirection,
    double value,
    Thumb thumb,
  });
}

/// Base class for [RangeSlider] tick mark shapes.
///
/// This is a simplified version of [SliderComponentShape] with a
/// [SliderThemeData] passed when getting the preferred size.
///
/// See also:
///
///  * [RoundRangeSliderTickMarkShape] for the default [RangeSlider]'s tick mark
///    shape that paints a solid circle.
///  * [RangeSliderThumbShape], which can be used to create custom shapes for
///    the [RangeSlider]'s thumb.
///  * [RangeSliderTrackShape], which can be used to create custom shapes for
///    the [RangeSlider]'s track.
///  * [RangeSliderValueIndicatorShape], which can be used to create custom
///    shapes for the [RangeSlider]'s value indicator.
///  * [SliderComponentShape], which can be used to create custom shapes for
///    the [Slider]'s thumb, overlay, and value indicator and the
///    [RangeSlider]'s overlay.
abstract class RangeSliderTickMarkShape {
  /// This abstract const constructor enables subclasses to provide
  /// const constructors so that they can be used in const expressions.
  const RangeSliderTickMarkShape();

  /// Returns the preferred size of the shape.
  ///
  /// It is used to help position the tick marks within the slider.
  ///
  /// {@macro flutter.material.RangeSliderThumbShape.paint.sliderTheme}
  ///
  /// {@macro flutter.material.RangeSliderThumbShape.getPreferredSize.isEnabled}
  Size getPreferredSize({required SliderThemeData sliderTheme, bool isEnabled});

  /// Paints the slider track.
  ///
  /// {@macro flutter.material.RangeSliderThumbShape.paint.context}
  ///
  /// {@macro flutter.material.SliderComponentShape.paint.center}
  ///
  /// {@macro flutter.material.RangeSliderValueIndicatorShape.paint.parentBox}
  ///
  /// {@macro flutter.material.RangeSliderThumbShape.paint.sliderTheme}
  ///
  /// {@macro flutter.material.RangeSliderThumbShape.paint.enableAnimation}
  ///
  /// {@macro flutter.material.RangeSliderThumbShape.getPreferredSize.isEnabled}
  ///
  /// The `textDirection` argument can be used to determine how the tick marks
  /// are painted depending on whether they are on an active track segment or not.
  ///
  /// {@template flutter.material.RangeSliderTickMarkShape.paint.trackSegment}
  /// The track segment between the two thumbs is the active track segment. The
  /// track segments between the thumb and each end of the slider are the inactive
  /// track segments. In [TextDirection.ltr], the start of the slider is on the
  /// left, and in [TextDirection.rtl], the start of the slider is on the right.
  /// {@endtemplate}
  void paint(
    PaintingContext context,
    Offset center, {
    required RenderBox parentBox,
    required SliderThemeData sliderTheme,
    required Animation<double> enableAnimation,
    required Offset startThumbCenter,
    required Offset endThumbCenter,
    bool isEnabled,
    required TextDirection textDirection,
  });
}

/// Base class for [RangeSlider] track shapes.
///
/// The slider's thumbs move along the track. A discrete slider's tick marks
/// are drawn after the track, but before the thumb, and are aligned with the
/// track.
///
/// The [getPreferredRect] helps position the slider thumbs and tick marks
/// relative to the track.
///
/// See also:
///
///  * [RoundedRectRangeSliderTrackShape] for the default [RangeSlider]'s track
///    shape that paints a stadium-like track.
///  * [RangeSliderTickMarkShape], which can be used to create custom shapes for
///    the [RangeSlider]'s tick marks.
///  * [RangeSliderThumbShape], which can be used to create custom shapes for
///    the [RangeSlider]'s thumb.
///  * [RangeSliderValueIndicatorShape], which can be used to create custom
///    shapes for the [RangeSlider]'s value indicator.
///  * [SliderComponentShape], which can be used to create custom shapes for
///    the [Slider]'s thumb, overlay, and value indicator and the
///    [RangeSlider]'s overlay.
abstract class RangeSliderTrackShape {
  /// This abstract const constructor enables subclasses to provide
  /// const constructors so that they can be used in const expressions.
  const RangeSliderTrackShape();

  /// Returns the preferred bounds of the shape.
  ///
  /// It is used to provide horizontal boundaries for the position of the
  /// thumbs, and to help position the slider thumbs and tick marks relative to
  /// the track.
  ///
  /// The `parentBox` argument can be used to help determine the preferredRect
  /// relative to attributes of the render box of the slider itself, such as
  /// size.
  ///
  /// The `offset` argument is relative to the caller's bounding box. It can be
  /// used to convert gesture coordinates from global to slider-relative
  /// coordinates.
  ///
  /// {@macro flutter.material.RangeSliderThumbShape.paint.sliderTheme}
  ///
  /// {@macro flutter.material.RangeSliderThumbShape.getPreferredSize.isEnabled}
  ///
  /// {@macro flutter.material.RangeSliderThumbShape.getPreferredSize.isDiscrete}
  Rect getPreferredRect({
    required RenderBox parentBox,
    Offset offset = Offset.zero,
    required SliderThemeData sliderTheme,
    bool isEnabled,
    bool isDiscrete,
  });

  /// Paints the track shape based on the state passed to it.
  ///
  /// {@macro flutter.material.SliderComponentShape.paint.context}
  ///
  /// The `offset` argument is the offset of the origin of the `parentBox` to
  /// the origin of its `context` canvas. This shape must be painted relative
  /// to this offset. See [PaintingContextCallback].
  ///
  /// {@macro flutter.material.RangeSliderValueIndicatorShape.paint.parentBox}
  ///
  /// {@macro flutter.material.RangeSliderThumbShape.paint.sliderTheme}
  ///
  /// {@macro flutter.material.RangeSliderThumbShape.paint.enableAnimation}
  ///
  /// The `startThumbCenter` argument is the offset of the center of the start
  /// thumb relative to the origin of the [PaintingContext.canvas]. It can be
  /// used as one point that divides the track between inactive and active.
  ///
  /// The `endThumbCenter` argument is the offset of the center of the end
  /// thumb relative to the origin of the [PaintingContext.canvas]. It can be
  /// used as one point that divides the track between inactive and active.
  ///
  /// {@macro flutter.material.RangeSliderThumbShape.getPreferredSize.isEnabled}
  ///
  /// {@macro flutter.material.RangeSliderThumbShape.getPreferredSize.isDiscrete}
  ///
  /// The `textDirection` argument can be used to determine how the track
  /// segments are painted depending on whether they are on an active track
  /// segment or not.
  ///
  /// {@macro flutter.material.RangeSliderTickMarkShape.paint.trackSegment}
  void paint(
    PaintingContext context,
    Offset offset, {
    required RenderBox parentBox,
    required SliderThemeData sliderTheme,
    required Animation<double> enableAnimation,
    required Offset startThumbCenter,
    required Offset endThumbCenter,
    bool isEnabled = false,
    bool isDiscrete = false,
    required TextDirection textDirection,
  });

  /// Whether the track shape is rounded. This is used to determine the correct
  /// position of the thumbs in relation to the track. Defaults to false.
  bool get isRounded => false;
}

/// Base track shape that provides an implementation of [getPreferredRect] for
/// default sizing.
///
/// The height is set from [SliderThemeData.trackHeight] and the width of the
/// parent box less the larger of the widths of [SliderThemeData.thumbShape] and
/// [SliderThemeData.overlayShape].
///
/// See also:
///
///  * [RectangularSliderTrackShape], which is a track shape with sharp
///    rectangular edges
///  * [RoundedRectSliderTrackShape], which is a track shape with round
///    stadium-like edges.
mixin BaseSliderTrackShape {
  /// Returns a rect that represents the track bounds that fits within the
  /// [Slider].
  ///
  /// The width is the width of the [Slider] or [RangeSlider], but padded by
  /// the max of the overlay and thumb radius. The height is defined by the
  /// [SliderThemeData.trackHeight].
  ///
  /// The [Rect] is centered both horizontally and vertically within the slider
  /// bounds.
  Rect getPreferredRect({
    required RenderBox parentBox,
    Offset offset = Offset.zero,
    required SliderThemeData sliderTheme,
    bool isEnabled = false,
    bool isDiscrete = false,
  }) {
    final double thumbWidth = sliderTheme.thumbShape!.getPreferredSize(isEnabled, isDiscrete).width;
    final double overlayWidth =
        sliderTheme.overlayShape!.getPreferredSize(isEnabled, isDiscrete).width;
    final double trackHeight = sliderTheme.trackHeight!;
    assert(overlayWidth >= 0);
    assert(trackHeight >= 0);

    final double trackLeft =
        offset.dx + (sliderTheme.padding == null ? math.max(overlayWidth / 2, thumbWidth / 2) : 0);
    final double trackTop = offset.dy + (parentBox.size.height - trackHeight) / 2;
    final double trackRight =
        trackLeft +
        parentBox.size.width -
        (sliderTheme.padding == null ? math.max(thumbWidth, overlayWidth) : 0);
    final double trackBottom = trackTop + trackHeight;
    // If the parentBox's size less than slider's size the trackRight will be less than trackLeft, so switch them.
    return Rect.fromLTRB(
      math.min(trackLeft, trackRight),
      trackTop,
      math.max(trackLeft, trackRight),
      trackBottom,
    );
  }

  /// Whether the track shape is rounded. This is used to determine the correct
  /// position of the thumb in relation to the track. Defaults to false.
  bool get isRounded => false;
}

/// A [Slider] track that's a simple rectangle.
///
/// It paints a solid colored rectangle, vertically centered in the
/// `parentBox`. The track rectangle extends to the bounds of the `parentBox`,
/// but is padded by the [RoundSliderOverlayShape] radius. The height is defined
/// by the [SliderThemeData.trackHeight]. The color is determined by the
/// [Slider]'s enabled state and the track segment's active state which are
/// defined by:
///   [SliderThemeData.activeTrackColor],
///   [SliderThemeData.inactiveTrackColor],
///   [SliderThemeData.disabledActiveTrackColor],
///   [SliderThemeData.disabledInactiveTrackColor].
///
/// {@macro flutter.material.SliderTrackShape.paint.trackSegment}
///
/// ![A slider widget, consisting of 5 divisions and showing the rectangular slider track shape.](https://flutter.github.io/assets-for-api-docs/assets/material/rectangular_slider_track_shape.png)
///
/// See also:
///
///  * [Slider], for the component that is meant to display this shape.
///  * [SliderThemeData], where an instance of this class is set to inform the
///    slider of the visual details of the its track.
///  * [SliderTrackShape], which can be used to create custom shapes for the
///    [Slider]'s track.
///  * [RoundedRectSliderTrackShape], for a similar track with rounded edges.
class RectangularSliderTrackShape extends SliderTrackShape with BaseSliderTrackShape {
  /// Creates a slider track that draws 2 rectangles.
  const RectangularSliderTrackShape();

  @override
  void paint(
    PaintingContext context,
    Offset offset, {
    required RenderBox parentBox,
    required SliderThemeData sliderTheme,
    required Animation<double> enableAnimation,
    required TextDirection textDirection,
    required Offset thumbCenter,
    Offset? secondaryOffset,
    bool isDiscrete = false,
    bool isEnabled = false,
  }) {
    assert(sliderTheme.disabledActiveTrackColor != null);
    assert(sliderTheme.disabledInactiveTrackColor != null);
    assert(sliderTheme.activeTrackColor != null);
    assert(sliderTheme.inactiveTrackColor != null);
    assert(sliderTheme.thumbShape != null);
    // If the slider [SliderThemeData.trackHeight] is less than or equal to 0,
    // then it makes no difference whether the track is painted or not,
    // therefore the painting can be a no-op.
    if (sliderTheme.trackHeight! <= 0) {
      return;
    }

    // Assign the track segment paints, which are left: active, right: inactive,
    // but reversed for right to left text.
    final ColorTween activeTrackColorTween = ColorTween(
      begin: sliderTheme.disabledActiveTrackColor,
      end: sliderTheme.activeTrackColor,
    );
    final ColorTween inactiveTrackColorTween = ColorTween(
      begin: sliderTheme.disabledInactiveTrackColor,
      end: sliderTheme.inactiveTrackColor,
    );
    final Paint activePaint = Paint()..color = activeTrackColorTween.evaluate(enableAnimation)!;
    final Paint inactivePaint = Paint()..color = inactiveTrackColorTween.evaluate(enableAnimation)!;
    final (Paint leftTrackPaint, Paint rightTrackPaint) = switch (textDirection) {
      TextDirection.ltr => (activePaint, inactivePaint),
      TextDirection.rtl => (inactivePaint, activePaint),
    };

    final Rect trackRect = getPreferredRect(
      parentBox: parentBox,
      offset: offset,
      sliderTheme: sliderTheme,
      isEnabled: isEnabled,
      isDiscrete: isDiscrete,
    );

    final Rect leftTrackSegment = Rect.fromLTRB(
      trackRect.left,
      trackRect.top,
      thumbCenter.dx,
      trackRect.bottom,
    );
    if (!leftTrackSegment.isEmpty) {
      context.canvas.drawRect(leftTrackSegment, leftTrackPaint);
    }
    final Rect rightTrackSegment = Rect.fromLTRB(
      thumbCenter.dx,
      trackRect.top,
      trackRect.right,
      trackRect.bottom,
    );
    if (!rightTrackSegment.isEmpty) {
      context.canvas.drawRect(rightTrackSegment, rightTrackPaint);
    }

    final bool showSecondaryTrack =
        secondaryOffset != null &&
        switch (textDirection) {
          TextDirection.rtl => secondaryOffset.dx < thumbCenter.dx,
          TextDirection.ltr => secondaryOffset.dx > thumbCenter.dx,
        };

    if (showSecondaryTrack) {
      final ColorTween secondaryTrackColorTween = ColorTween(
        begin: sliderTheme.disabledSecondaryActiveTrackColor,
        end: sliderTheme.secondaryActiveTrackColor,
      );
      final Paint secondaryTrackPaint =
          Paint()..color = secondaryTrackColorTween.evaluate(enableAnimation)!;
      final Rect secondaryTrackSegment = switch (textDirection) {
        TextDirection.rtl => Rect.fromLTRB(
          secondaryOffset.dx,
          trackRect.top,
          thumbCenter.dx,
          trackRect.bottom,
        ),
        TextDirection.ltr => Rect.fromLTRB(
          thumbCenter.dx,
          trackRect.top,
          secondaryOffset.dx,
          trackRect.bottom,
        ),
      };
      if (!secondaryTrackSegment.isEmpty) {
        context.canvas.drawRect(secondaryTrackSegment, secondaryTrackPaint);
      }
    }
  }
}

/// The default shape of a [Slider]'s track.
///
/// It paints a solid colored rectangle with rounded edges, vertically centered
/// in the `parentBox`. The track rectangle extends to the bounds of the
/// `parentBox`, but is padded by the larger of [RoundSliderOverlayShape]'s
/// radius and [RoundSliderThumbShape]'s radius. The height is defined by the
/// [SliderThemeData.trackHeight]. The color is determined by the [Slider]'s
/// enabled state and the track segment's active state which are defined by:
///   [SliderThemeData.activeTrackColor],
///   [SliderThemeData.inactiveTrackColor],
///   [SliderThemeData.disabledActiveTrackColor],
///   [SliderThemeData.disabledInactiveTrackColor].
///
/// {@macro flutter.material.SliderTrackShape.paint.trackSegment}
///
/// ![A slider widget, consisting of 5 divisions and showing the rounded rect slider track shape.](https://flutter.github.io/assets-for-api-docs/assets/material/rounded_rect_slider_track_shape.png)
///
/// See also:
///
///  * [Slider], for the component that is meant to display this shape.
///  * [SliderThemeData], where an instance of this class is set to inform the
///    slider of the visual details of the its track.
///  * [SliderTrackShape], which can be used to create custom shapes for the
///    [Slider]'s track.
///  * [RectangularSliderTrackShape], for a similar track with sharp edges.
class RoundedRectSliderTrackShape extends SliderTrackShape with BaseSliderTrackShape {
  /// Create a slider track that draws two rectangles with rounded outer edges.
  const RoundedRectSliderTrackShape();

  @override
  void paint(
    PaintingContext context,
    Offset offset, {
    required RenderBox parentBox,
    required SliderThemeData sliderTheme,
    required Animation<double> enableAnimation,
    required TextDirection textDirection,
    required Offset thumbCenter,
    Offset? secondaryOffset,
    bool isDiscrete = false,
    bool isEnabled = false,
    double additionalActiveTrackHeight = 2,
  }) {
    assert(sliderTheme.disabledActiveTrackColor != null);
    assert(sliderTheme.disabledInactiveTrackColor != null);
    assert(sliderTheme.activeTrackColor != null);
    assert(sliderTheme.inactiveTrackColor != null);
    assert(sliderTheme.thumbShape != null);
    // If the slider [SliderThemeData.trackHeight] is less than or equal to 0,
    // then it makes no difference whether the track is painted or not,
    // therefore the painting can be a no-op.
    if (sliderTheme.trackHeight == null || sliderTheme.trackHeight! <= 0) {
      return;
    }

    // Assign the track segment paints, which are leading: active and
    // trailing: inactive.
    final ColorTween activeTrackColorTween = ColorTween(
      begin: sliderTheme.disabledActiveTrackColor,
      end: sliderTheme.activeTrackColor,
    );
    final ColorTween inactiveTrackColorTween = ColorTween(
      begin: sliderTheme.disabledInactiveTrackColor,
      end: sliderTheme.inactiveTrackColor,
    );
    final Paint activePaint = Paint()..color = activeTrackColorTween.evaluate(enableAnimation)!;
    final Paint inactivePaint = Paint()..color = inactiveTrackColorTween.evaluate(enableAnimation)!;
    final (Paint leftTrackPaint, Paint rightTrackPaint) = switch (textDirection) {
      TextDirection.ltr => (activePaint, inactivePaint),
      TextDirection.rtl => (inactivePaint, activePaint),
    };

    final Rect trackRect = getPreferredRect(
      parentBox: parentBox,
      offset: offset,
      sliderTheme: sliderTheme,
      isEnabled: isEnabled,
      isDiscrete: isDiscrete,
    );
    final Radius trackRadius = Radius.circular(trackRect.height / 2);
    final Radius activeTrackRadius = Radius.circular(
      (trackRect.height + additionalActiveTrackHeight) / 2,
    );
    final bool isLTR = textDirection == TextDirection.ltr;
    final bool isRTL = textDirection == TextDirection.rtl;

    final bool drawInactiveTrack =
        thumbCenter.dx < (trackRect.right - (sliderTheme.trackHeight! / 2));
    if (drawInactiveTrack) {
      // Draw the inactive track segment.
      context.canvas.drawRRect(
        RRect.fromLTRBR(
          thumbCenter.dx - (sliderTheme.trackHeight! / 2),
          isRTL ? trackRect.top - (additionalActiveTrackHeight / 2) : trackRect.top,
          trackRect.right,
          isRTL ? trackRect.bottom + (additionalActiveTrackHeight / 2) : trackRect.bottom,
          isLTR ? trackRadius : activeTrackRadius,
        ),
        rightTrackPaint,
      );
    }
    final bool drawActiveTrack = thumbCenter.dx > (trackRect.left + (sliderTheme.trackHeight! / 2));
    if (drawActiveTrack) {
      // Draw the active track segment.
      context.canvas.drawRRect(
        RRect.fromLTRBR(
          trackRect.left,
          isLTR ? trackRect.top - (additionalActiveTrackHeight / 2) : trackRect.top,
          thumbCenter.dx + (sliderTheme.trackHeight! / 2),
          isLTR ? trackRect.bottom + (additionalActiveTrackHeight / 2) : trackRect.bottom,
          isLTR ? activeTrackRadius : trackRadius,
        ),
        leftTrackPaint,
      );
    }

    final bool showSecondaryTrack =
        (secondaryOffset != null) &&
        (isLTR ? (secondaryOffset.dx > thumbCenter.dx) : (secondaryOffset.dx < thumbCenter.dx));

    if (showSecondaryTrack) {
      final ColorTween secondaryTrackColorTween = ColorTween(
        begin: sliderTheme.disabledSecondaryActiveTrackColor,
        end: sliderTheme.secondaryActiveTrackColor,
      );
      final Paint secondaryTrackPaint =
          Paint()..color = secondaryTrackColorTween.evaluate(enableAnimation)!;
      if (isLTR) {
        context.canvas.drawRRect(
          RRect.fromLTRBAndCorners(
            thumbCenter.dx,
            trackRect.top,
            secondaryOffset.dx,
            trackRect.bottom,
            topRight: trackRadius,
            bottomRight: trackRadius,
          ),
          secondaryTrackPaint,
        );
      } else {
        context.canvas.drawRRect(
          RRect.fromLTRBAndCorners(
            secondaryOffset.dx,
            trackRect.top,
            thumbCenter.dx,
            trackRect.bottom,
            topLeft: trackRadius,
            bottomLeft: trackRadius,
          ),
          secondaryTrackPaint,
        );
      }
    }
  }

  @override
  bool get isRounded => true;
}

/// Base range slider track shape that provides an implementation of [getPreferredRect] for
/// default sizing.
///
/// The height is set from [SliderThemeData.trackHeight] and the width of the
/// parent box less the larger of the widths of [SliderThemeData.rangeThumbShape] and
/// [SliderThemeData.overlayShape].
///
/// See also:
///
///  * [RectangularRangeSliderTrackShape], which is a track shape with sharp
///    rectangular edges
mixin BaseRangeSliderTrackShape {
  /// Returns a rect that represents the track bounds that fits within the
  /// [Slider].
  ///
  /// The width is the width of the [RangeSlider], but padded by the max
  /// of the overlay and thumb radius. The height is defined by the  [SliderThemeData.trackHeight].
  ///
  /// The [Rect] is centered both horizontally and vertically within the slider
  /// bounds.
  Rect getPreferredRect({
    required RenderBox parentBox,
    Offset offset = Offset.zero,
    required SliderThemeData sliderTheme,
    bool isEnabled = false,
    bool isDiscrete = false,
  }) {
    assert(sliderTheme.rangeThumbShape != null);
    assert(sliderTheme.overlayShape != null);
    assert(sliderTheme.trackHeight != null);
    final double thumbWidth =
        sliderTheme.rangeThumbShape!.getPreferredSize(isEnabled, isDiscrete).width;
    final double overlayWidth =
        sliderTheme.overlayShape!.getPreferredSize(isEnabled, isDiscrete).width;
    final double trackHeight = sliderTheme.trackHeight!;
    assert(overlayWidth >= 0);
    assert(trackHeight >= 0);

    final double trackLeft = offset.dx + math.max(overlayWidth / 2, thumbWidth / 2);
    final double trackTop = offset.dy + (parentBox.size.height - trackHeight) / 2;
    final double trackRight = trackLeft + parentBox.size.width - math.max(thumbWidth, overlayWidth);
    final double trackBottom = trackTop + trackHeight;
    // If the parentBox's size less than slider's size the trackRight will be less than trackLeft, so switch them.
    return Rect.fromLTRB(
      math.min(trackLeft, trackRight),
      trackTop,
      math.max(trackLeft, trackRight),
      trackBottom,
    );
  }
}

/// A [RangeSlider] track that's a simple rectangle.
///
/// It paints a solid colored rectangle, vertically centered in the
/// `parentBox`. The track rectangle extends to the bounds of the `parentBox`,
/// but is padded by the [RoundSliderOverlayShape] radius. The height is
/// defined by the [SliderThemeData.trackHeight]. The color is determined by the
/// [Slider]'s enabled state and the track segment's active state which are
/// defined by:
///   [SliderThemeData.activeTrackColor],
///   [SliderThemeData.inactiveTrackColor],
///   [SliderThemeData.disabledActiveTrackColor],
///   [SliderThemeData.disabledInactiveTrackColor].
///
/// {@macro flutter.material.RangeSliderTickMarkShape.paint.trackSegment}
///
/// ![A range slider widget, consisting of 5 divisions and showing the rectangular range slider track shape.](https://flutter.github.io/assets-for-api-docs/assets/material/rectangular_range_slider_track_shape.png)
///
/// See also:
///
///  * [RangeSlider], for the component that is meant to display this shape.
///  * [SliderThemeData], where an instance of this class is set to inform the
///    slider of the visual details of the its track.
///  * [RangeSliderTrackShape], which can be used to create custom shapes for
///    the [RangeSlider]'s track.
///  * [RoundedRectRangeSliderTrackShape], for a similar track with rounded
///    edges.
class RectangularRangeSliderTrackShape extends RangeSliderTrackShape
    with BaseRangeSliderTrackShape {
  /// Create a slider track with rectangular outer edges.
  ///
  /// The middle track segment is the selected range and is active, and the two
  /// outer track segments are inactive.
  const RectangularRangeSliderTrackShape();

  @override
  void paint(
    PaintingContext context,
    Offset offset, {
    required RenderBox parentBox,
    required SliderThemeData sliderTheme,
    required Animation<double>? enableAnimation,
    required Offset startThumbCenter,
    required Offset endThumbCenter,
    bool isEnabled = false,
    bool isDiscrete = false,
    required TextDirection textDirection,
  }) {
    assert(sliderTheme.disabledActiveTrackColor != null);
    assert(sliderTheme.disabledInactiveTrackColor != null);
    assert(sliderTheme.activeTrackColor != null);
    assert(sliderTheme.inactiveTrackColor != null);
    assert(sliderTheme.rangeThumbShape != null);
    assert(enableAnimation != null);
    // Assign the track segment paints, which are left: active, right: inactive,
    // but reversed for right to left text.
    final ColorTween activeTrackColorTween = ColorTween(
      begin: sliderTheme.disabledActiveTrackColor,
      end: sliderTheme.activeTrackColor,
    );
    final ColorTween inactiveTrackColorTween = ColorTween(
      begin: sliderTheme.disabledInactiveTrackColor,
      end: sliderTheme.inactiveTrackColor,
    );
    final Paint activePaint = Paint()..color = activeTrackColorTween.evaluate(enableAnimation!)!;
    final Paint inactivePaint = Paint()..color = inactiveTrackColorTween.evaluate(enableAnimation)!;

    final (Offset leftThumbOffset, Offset rightThumbOffset) = switch (textDirection) {
      TextDirection.ltr => (startThumbCenter, endThumbCenter),
      TextDirection.rtl => (endThumbCenter, startThumbCenter),
    };

    final Rect trackRect = getPreferredRect(
      parentBox: parentBox,
      offset: offset,
      sliderTheme: sliderTheme,
      isEnabled: isEnabled,
      isDiscrete: isDiscrete,
    );
    final Rect leftTrackSegment = Rect.fromLTRB(
      trackRect.left,
      trackRect.top,
      leftThumbOffset.dx,
      trackRect.bottom,
    );
    if (!leftTrackSegment.isEmpty) {
      context.canvas.drawRect(leftTrackSegment, inactivePaint);
    }
    final Rect middleTrackSegment = Rect.fromLTRB(
      leftThumbOffset.dx,
      trackRect.top,
      rightThumbOffset.dx,
      trackRect.bottom,
    );
    if (!middleTrackSegment.isEmpty) {
      context.canvas.drawRect(middleTrackSegment, activePaint);
    }
    final Rect rightTrackSegment = Rect.fromLTRB(
      rightThumbOffset.dx,
      trackRect.top,
      trackRect.right,
      trackRect.bottom,
    );
    if (!rightTrackSegment.isEmpty) {
      context.canvas.drawRect(rightTrackSegment, inactivePaint);
    }
  }
}

/// The default shape of a [RangeSlider]'s track.
///
/// It paints a solid colored rectangle with rounded edges, vertically centered
/// in the `parentBox`. The track rectangle extends to the bounds of the
/// `parentBox`, but is padded by the larger of [RoundSliderOverlayShape]'s
/// radius and [RoundRangeSliderThumbShape]'s radius. The height is defined by
/// the [SliderThemeData.trackHeight]. The color is determined by the
/// [RangeSlider]'s enabled state and the track segment's active state which are
/// defined by:
///   [SliderThemeData.activeTrackColor],
///   [SliderThemeData.inactiveTrackColor],
///   [SliderThemeData.disabledActiveTrackColor],
///   [SliderThemeData.disabledInactiveTrackColor].
///
/// {@macro flutter.material.RangeSliderTickMarkShape.paint.trackSegment}
///
/// ![A range slider widget, consisting of 5 divisions and showing the rounded rect range slider track shape.](https://flutter.github.io/assets-for-api-docs/assets/material/rounded_rect_range_slider_track_shape.png)
///
/// See also:
///
///  * [RangeSlider], for the component that is meant to display this shape.
///  * [SliderThemeData], where an instance of this class is set to inform the
///    slider of the visual details of the its track.
///  * [RangeSliderTrackShape], which can be used to create custom shapes for
///    the [RangeSlider]'s track.
///  * [RectangularRangeSliderTrackShape], for a similar track with sharp edges.
class RoundedRectRangeSliderTrackShape extends RangeSliderTrackShape
    with BaseRangeSliderTrackShape {
  /// Create a slider track with rounded outer edges.
  ///
  /// The middle track segment is the selected range and is active, and the two
  /// outer track segments are inactive.
  const RoundedRectRangeSliderTrackShape();

  @override
  void paint(
    PaintingContext context,
    Offset offset, {
    required RenderBox parentBox,
    required SliderThemeData sliderTheme,
    required Animation<double> enableAnimation,
    required Offset startThumbCenter,
    required Offset endThumbCenter,
    bool isEnabled = false,
    bool isDiscrete = false,
    required TextDirection textDirection,
    double additionalActiveTrackHeight = 2,
  }) {
    assert(sliderTheme.disabledActiveTrackColor != null);
    assert(sliderTheme.disabledInactiveTrackColor != null);
    assert(sliderTheme.activeTrackColor != null);
    assert(sliderTheme.inactiveTrackColor != null);
    assert(sliderTheme.rangeThumbShape != null);

    if (sliderTheme.trackHeight == null || sliderTheme.trackHeight! <= 0) {
      return;
    }

    // Assign the track segment paints, which are left: active, right: inactive,
    // but reversed for right to left text.
    final ColorTween activeTrackColorTween = ColorTween(
      begin: sliderTheme.disabledActiveTrackColor,
      end: sliderTheme.activeTrackColor,
    );
    final ColorTween inactiveTrackColorTween = ColorTween(
      begin: sliderTheme.disabledInactiveTrackColor,
      end: sliderTheme.inactiveTrackColor,
    );
    final Paint activePaint = Paint()..color = activeTrackColorTween.evaluate(enableAnimation)!;
    final Paint inactivePaint = Paint()..color = inactiveTrackColorTween.evaluate(enableAnimation)!;

    final (Offset leftThumbOffset, Offset rightThumbOffset) = switch (textDirection) {
      TextDirection.ltr => (startThumbCenter, endThumbCenter),
      TextDirection.rtl => (endThumbCenter, startThumbCenter),
    };
    final Size thumbSize = sliderTheme.rangeThumbShape!.getPreferredSize(isEnabled, isDiscrete);
    final double thumbRadius = thumbSize.width / 2;
    assert(thumbRadius > 0);

    final Rect trackRect = getPreferredRect(
      parentBox: parentBox,
      offset: offset,
      sliderTheme: sliderTheme,
      isEnabled: isEnabled,
      isDiscrete: isDiscrete,
    );

    final Radius trackRadius = Radius.circular(trackRect.height / 2);

    context.canvas.drawRRect(
      RRect.fromLTRBAndCorners(
        trackRect.left,
        trackRect.top,
        leftThumbOffset.dx,
        trackRect.bottom,
        topLeft: trackRadius,
        bottomLeft: trackRadius,
      ),
      inactivePaint,
    );
    context.canvas.drawRRect(
      RRect.fromLTRBAndCorners(
        rightThumbOffset.dx,
        trackRect.top,
        trackRect.right,
        trackRect.bottom,
        topRight: trackRadius,
        bottomRight: trackRadius,
      ),
      inactivePaint,
    );
    context.canvas.drawRRect(
      RRect.fromLTRBR(
        leftThumbOffset.dx - (sliderTheme.trackHeight! / 2),
        trackRect.top - (additionalActiveTrackHeight / 2),
        rightThumbOffset.dx + (sliderTheme.trackHeight! / 2),
        trackRect.bottom + (additionalActiveTrackHeight / 2),
        trackRadius,
      ),
      activePaint,
    );
  }

  @override
  bool get isRounded => true;
}

/// The default shape of each [Slider] tick mark.
///
/// Tick marks are only displayed if the slider is discrete, which can be done
/// by setting the [Slider.divisions] to an integer value.
///
/// It paints a solid circle, centered in the on the track.
/// The color is determined by the [Slider]'s enabled state and track's active
/// states. These colors are defined in:
///   [SliderThemeData.activeTrackColor],
///   [SliderThemeData.inactiveTrackColor],
///   [SliderThemeData.disabledActiveTrackColor],
///   [SliderThemeData.disabledInactiveTrackColor].
///
/// ![A slider widget, consisting of 5 divisions and showing the round slider tick mark shape.](https://flutter.github.io/assets-for-api-docs/assets/material/rounded_slider_tick_mark_shape.png)
///
/// See also:
///
///  * [Slider], which includes tick marks defined by this shape.
///  * [SliderTheme], which can be used to configure the tick mark shape of all
///    sliders in a widget subtree.
class RoundSliderTickMarkShape extends SliderTickMarkShape {
  /// Create a slider tick mark that draws a circle.
  const RoundSliderTickMarkShape({this.tickMarkRadius});

  /// The preferred radius of the round tick mark.
  ///
  /// If it is not provided, then 1/4 of the [SliderThemeData.trackHeight] is used.
  final double? tickMarkRadius;

  @override
  Size getPreferredSize({required SliderThemeData sliderTheme, required bool isEnabled}) {
    assert(sliderTheme.trackHeight != null);
    // The tick marks are tiny circles. If no radius is provided, then the
    // radius is defaulted to be a fraction of the
    // [SliderThemeData.trackHeight]. The fraction is 1/4.
    return Size.fromRadius(tickMarkRadius ?? sliderTheme.trackHeight! / 4);
  }

  @override
  void paint(
    PaintingContext context,
    Offset center, {
    required RenderBox parentBox,
    required SliderThemeData sliderTheme,
    required Animation<double> enableAnimation,
    required TextDirection textDirection,
    required Offset thumbCenter,
    required bool isEnabled,
  }) {
    assert(sliderTheme.disabledActiveTickMarkColor != null);
    assert(sliderTheme.disabledInactiveTickMarkColor != null);
    assert(sliderTheme.activeTickMarkColor != null);
    assert(sliderTheme.inactiveTickMarkColor != null);
    // The paint color of the tick mark depends on its position relative
    // to the thumb and the text direction.
    final double xOffset = center.dx - thumbCenter.dx;
    final (Color? begin, Color? end) = switch (textDirection) {
      TextDirection.ltr when xOffset > 0 => (
        sliderTheme.disabledInactiveTickMarkColor,
        sliderTheme.inactiveTickMarkColor,
      ),
      TextDirection.rtl when xOffset < 0 => (
        sliderTheme.disabledInactiveTickMarkColor,
        sliderTheme.inactiveTickMarkColor,
      ),
      TextDirection.ltr || TextDirection.rtl => (
        sliderTheme.disabledActiveTickMarkColor,
        sliderTheme.activeTickMarkColor,
      ),
    };
    final Paint paint =
        Paint()..color = ColorTween(begin: begin, end: end).evaluate(enableAnimation)!;

    // The tick marks are tiny circles that are the same height as the track.
    final double tickMarkRadius =
        getPreferredSize(isEnabled: isEnabled, sliderTheme: sliderTheme).width / 2;
    if (tickMarkRadius > 0) {
      context.canvas.drawCircle(center, tickMarkRadius, paint);
    }
  }
}

/// The default shape of each [RangeSlider] tick mark.
///
/// Tick marks are only displayed if the slider is discrete, which can be done
/// by setting the [RangeSlider.divisions] to an integer value.
///
/// It paints a solid circle, centered on the track.
/// The color is determined by the [Slider]'s enabled state and track's active
/// states. These colors are defined in:
///   [SliderThemeData.activeTrackColor],
///   [SliderThemeData.inactiveTrackColor],
///   [SliderThemeData.disabledActiveTrackColor],
///   [SliderThemeData.disabledInactiveTrackColor].
///
/// ![A slider widget, consisting of 5 divisions and showing the round range slider tick mark shape.](https://flutter.github.io/assets-for-api-docs/assets/material/round_range_slider_tick_mark_shape.png)
///
/// See also:
///
///  * [RangeSlider], which includes tick marks defined by this shape.
///  * [SliderTheme], which can be used to configure the tick mark shape of all
///    sliders in a widget subtree.
class RoundRangeSliderTickMarkShape extends RangeSliderTickMarkShape {
  /// Create a range slider tick mark that draws a circle.
  const RoundRangeSliderTickMarkShape({this.tickMarkRadius});

  /// The preferred radius of the round tick mark.
  ///
  /// If it is not provided, then 1/4 of the [SliderThemeData.trackHeight] is used.
  final double? tickMarkRadius;

  @override
  Size getPreferredSize({required SliderThemeData sliderTheme, bool isEnabled = false}) {
    assert(sliderTheme.trackHeight != null);
    return Size.fromRadius(tickMarkRadius ?? sliderTheme.trackHeight! / 4);
  }

  @override
  void paint(
    PaintingContext context,
    Offset center, {
    required RenderBox parentBox,
    required SliderThemeData sliderTheme,
    required Animation<double> enableAnimation,
    required Offset startThumbCenter,
    required Offset endThumbCenter,
    bool isEnabled = false,
    required TextDirection textDirection,
  }) {
    assert(sliderTheme.disabledActiveTickMarkColor != null);
    assert(sliderTheme.disabledInactiveTickMarkColor != null);
    assert(sliderTheme.activeTickMarkColor != null);
    assert(sliderTheme.inactiveTickMarkColor != null);

    final bool isBetweenThumbs = switch (textDirection) {
      TextDirection.ltr => startThumbCenter.dx < center.dx && center.dx < endThumbCenter.dx,
      TextDirection.rtl => endThumbCenter.dx < center.dx && center.dx < startThumbCenter.dx,
    };
    final Color? begin =
        isBetweenThumbs
            ? sliderTheme.disabledActiveTickMarkColor
            : sliderTheme.disabledInactiveTickMarkColor;
    final Color? end =
        isBetweenThumbs ? sliderTheme.activeTickMarkColor : sliderTheme.inactiveTickMarkColor;
    final Paint paint =
        Paint()..color = ColorTween(begin: begin, end: end).evaluate(enableAnimation)!;

    // The tick marks are tiny circles that are the same height as the track.
    final double tickMarkRadius =
        getPreferredSize(isEnabled: isEnabled, sliderTheme: sliderTheme).width / 2;
    if (tickMarkRadius > 0) {
      context.canvas.drawCircle(center, tickMarkRadius, paint);
    }
  }
}

/// A special version of [SliderTickMarkShape] that has a zero size and paints
/// nothing.
///
/// This class is used to create a special instance of a [SliderTickMarkShape]
/// that will not paint any tick mark shape. A static reference is stored in
/// [SliderTickMarkShape.noTickMark]. When this value is specified for
/// [SliderThemeData.tickMarkShape], the tick mark painting is skipped.
class _EmptySliderTickMarkShape extends SliderTickMarkShape {
  @override
  Size getPreferredSize({required SliderThemeData sliderTheme, required bool isEnabled}) {
    return Size.zero;
  }

  @override
  void paint(
    PaintingContext context,
    Offset center, {
    required RenderBox parentBox,
    required SliderThemeData sliderTheme,
    required Animation<double> enableAnimation,
    required Offset thumbCenter,
    required bool isEnabled,
    required TextDirection textDirection,
  }) {
    // no-op.
  }
}

/// A special version of [SliderComponentShape] that has a zero size and paints
/// nothing.
///
/// This class is used to create a special instance of a [SliderComponentShape]
/// that will not paint any component shape. A static reference is stored in
/// [SliderComponentShape.noThumb] and [SliderComponentShape.noOverlay]. When this value
/// is specified for [SliderThemeData.thumbShape], the thumb painting is
/// skipped. When this value is specified for [SliderThemeData.overlayShape],
/// the overlay painting is skipped.
class _EmptySliderComponentShape extends SliderComponentShape {
  @override
  Size getPreferredSize(bool isEnabled, bool isDiscrete) => Size.zero;

  @override
  void paint(
    PaintingContext context,
    Offset center, {
    required Animation<double> activationAnimation,
    required Animation<double> enableAnimation,
    required bool isDiscrete,
    required TextPainter labelPainter,
    required RenderBox parentBox,
    required SliderThemeData sliderTheme,
    required TextDirection textDirection,
    required double value,
    required double textScaleFactor,
    required Size sizeWithOverflow,
  }) {
    // no-op.
  }
}

/// The default shape of a [Slider]'s thumb.
///
/// There is a shadow for the resting, pressed, hovered, and focused state.
///
/// ![A slider widget, consisting of 5 divisions and showing the round slider thumb shape.](https://flutter.github.io/assets-for-api-docs/assets/material/round_slider_thumb_shape.png)
///
/// See also:
///
///  * [Slider], which includes a thumb defined by this shape.
///  * [SliderTheme], which can be used to configure the thumb shape of all
///    sliders in a widget subtree.
class RoundSliderThumbShape extends SliderComponentShape {
  /// Create a slider thumb that draws a circle.
  const RoundSliderThumbShape({
    this.enabledThumbRadius = 10.0,
    this.disabledThumbRadius,
    this.elevation = 1.0,
    this.pressedElevation = 6.0,
  });

  /// The preferred radius of the round thumb shape when the slider is enabled.
  ///
  /// If it is not provided, then the Material Design default of 10 is used.
  final double enabledThumbRadius;

  /// The preferred radius of the round thumb shape when the slider is disabled.
  ///
  /// If no disabledRadius is provided, then it is equal to the
  /// [enabledThumbRadius]
  final double? disabledThumbRadius;
  double get _disabledThumbRadius => disabledThumbRadius ?? enabledThumbRadius;

  /// The resting elevation adds shadow to the unpressed thumb.
  ///
  /// The default is 1.
  ///
  /// Use 0 for no shadow. The higher the value, the larger the shadow. For
  /// example, a value of 12 will create a very large shadow.
  ///
  final double elevation;

  /// The pressed elevation adds shadow to the pressed thumb.
  ///
  /// The default is 6.
  ///
  /// Use 0 for no shadow. The higher the value, the larger the shadow. For
  /// example, a value of 12 will create a very large shadow.
  final double pressedElevation;

  @override
  Size getPreferredSize(bool isEnabled, bool isDiscrete) {
    return Size.fromRadius(isEnabled ? enabledThumbRadius : _disabledThumbRadius);
  }

  @override
  void paint(
    PaintingContext context,
    Offset center, {
    required Animation<double> activationAnimation,
    required Animation<double> enableAnimation,
    required bool isDiscrete,
    required TextPainter labelPainter,
    required RenderBox parentBox,
    required SliderThemeData sliderTheme,
    required TextDirection textDirection,
    required double value,
    required double textScaleFactor,
    required Size sizeWithOverflow,
  }) {
    assert(sliderTheme.disabledThumbColor != null);
    assert(sliderTheme.thumbColor != null);

    final Canvas canvas = context.canvas;
    final Tween<double> radiusTween = Tween<double>(
      begin: _disabledThumbRadius,
      end: enabledThumbRadius,
    );
    final ColorTween colorTween = ColorTween(
      begin: sliderTheme.disabledThumbColor,
      end: sliderTheme.thumbColor,
    );

    final Color color = colorTween.evaluate(enableAnimation)!;
    final double radius = radiusTween.evaluate(enableAnimation);

    final Tween<double> elevationTween = Tween<double>(begin: elevation, end: pressedElevation);

    final double evaluatedElevation = elevationTween.evaluate(activationAnimation);
    final Path path =
        Path()..addArc(
          Rect.fromCenter(center: center, width: 2 * radius, height: 2 * radius),
          0,
          math.pi * 2,
        );

    bool paintShadows = true;
    assert(() {
      if (debugDisableShadows) {
        _debugDrawShadow(canvas, path, evaluatedElevation);
        paintShadows = false;
      }
      return true;
    }());

    if (paintShadows) {
      canvas.drawShadow(path, Colors.black, evaluatedElevation, true);
    }

    canvas.drawCircle(center, radius, Paint()..color = color);
  }
}

/// The default shape of a [RangeSlider]'s thumbs.
///
/// There is a shadow for the resting and pressed state.
///
/// ![A slider widget, consisting of 5 divisions and showing the round range slider thumb shape.](https://flutter.github.io/assets-for-api-docs/assets/material/round_range_slider_thumb_shape.png)
///
/// See also:
///
///  * [RangeSlider], which includes thumbs defined by this shape.
///  * [SliderTheme], which can be used to configure the thumb shapes of all
///    range sliders in a widget subtree.
class RoundRangeSliderThumbShape extends RangeSliderThumbShape {
  /// Create a slider thumb that draws a circle.
  const RoundRangeSliderThumbShape({
    this.enabledThumbRadius = 10.0,
    this.disabledThumbRadius,
    this.elevation = 1.0,
    this.pressedElevation = 6.0,
  });

  /// The preferred radius of the round thumb shape when the slider is enabled.
  ///
  /// If it is not provided, then the Material Design default of 10 is used.
  final double enabledThumbRadius;

  /// The preferred radius of the round thumb shape when the slider is disabled.
  ///
  /// If no disabledRadius is provided, then it is equal to the
  /// [enabledThumbRadius].
  final double? disabledThumbRadius;
  double get _disabledThumbRadius => disabledThumbRadius ?? enabledThumbRadius;

  /// The resting elevation adds shadow to the unpressed thumb.
  ///
  /// The default is 1.
  final double elevation;

  /// The pressed elevation adds shadow to the pressed thumb.
  ///
  /// The default is 6.
  final double pressedElevation;

  @override
  Size getPreferredSize(bool isEnabled, bool isDiscrete) {
    return Size.fromRadius(isEnabled ? enabledThumbRadius : _disabledThumbRadius);
  }

  @override
  void paint(
    PaintingContext context,
    Offset center, {
    required Animation<double> activationAnimation,
    required Animation<double> enableAnimation,
    bool isDiscrete = false,
    bool isEnabled = false,
    bool? isOnTop,
    required SliderThemeData sliderTheme,
    TextDirection? textDirection,
    Thumb? thumb,
    bool? isPressed,
  }) {
    assert(sliderTheme.showValueIndicator != null);
    assert(sliderTheme.overlappingShapeStrokeColor != null);
    final Canvas canvas = context.canvas;
    final Tween<double> radiusTween = Tween<double>(
      begin: _disabledThumbRadius,
      end: enabledThumbRadius,
    );
    final ColorTween colorTween = ColorTween(
      begin: sliderTheme.disabledThumbColor,
      end: sliderTheme.thumbColor,
    );
    final double radius = radiusTween.evaluate(enableAnimation);
    final Tween<double> elevationTween = Tween<double>(begin: elevation, end: pressedElevation);

    // Add a stroke of 1dp around the circle if this thumb would overlap
    // the other thumb.
    if (isOnTop ?? false) {
      final Paint strokePaint =
          Paint()
            ..color = sliderTheme.overlappingShapeStrokeColor!
            ..strokeWidth = 1.0
            ..style = PaintingStyle.stroke;
      canvas.drawCircle(center, radius, strokePaint);
    }

    final Color color = colorTween.evaluate(enableAnimation)!;

    final double evaluatedElevation =
        isPressed! ? elevationTween.evaluate(activationAnimation) : elevation;
    final Path shadowPath =
        Path()..addArc(
          Rect.fromCenter(center: center, width: 2 * radius, height: 2 * radius),
          0,
          math.pi * 2,
        );

    bool paintShadows = true;
    assert(() {
      if (debugDisableShadows) {
        _debugDrawShadow(canvas, shadowPath, evaluatedElevation);
        paintShadows = false;
      }
      return true;
    }());

    if (paintShadows) {
      canvas.drawShadow(shadowPath, Colors.black, evaluatedElevation, true);
    }

    canvas.drawCircle(center, radius, Paint()..color = color);
  }
}

/// The default shape of a [Slider]'s thumb overlay.
///
/// The shape of the overlay is a circle with the same center as the thumb, but
/// with a larger radius. It animates to full size when the thumb is pressed,
/// and animates back down to size 0 when it is released. It is painted behind
/// the thumb, and is expected to extend beyond the bounds of the thumb so that
/// it is visible.
///
/// The overlay color is defined by [SliderThemeData.overlayColor].
///
/// See also:
///
///  * [Slider], which includes an overlay defined by this shape.
///  * [SliderTheme], which can be used to configure the overlay shape of all
///    sliders in a widget subtree.
class RoundSliderOverlayShape extends SliderComponentShape {
  /// Create a slider thumb overlay that draws a circle.
  const RoundSliderOverlayShape({this.overlayRadius = 24.0});

  /// The preferred radius of the round thumb shape when enabled.
  ///
  /// If it is not provided, then half of the [SliderThemeData.trackHeight] is
  /// used.
  final double overlayRadius;

  @override
  Size getPreferredSize(bool isEnabled, bool isDiscrete) {
    return Size.fromRadius(overlayRadius);
  }

  @override
  void paint(
    PaintingContext context,
    Offset center, {
    required Animation<double> activationAnimation,
    required Animation<double> enableAnimation,
    required bool isDiscrete,
    required TextPainter labelPainter,
    required RenderBox parentBox,
    required SliderThemeData sliderTheme,
    required TextDirection textDirection,
    required double value,
    required double textScaleFactor,
    required Size sizeWithOverflow,
  }) {
    final Canvas canvas = context.canvas;
    final Tween<double> radiusTween = Tween<double>(begin: 0.0, end: overlayRadius);

    canvas.drawCircle(
      center,
      radiusTween.evaluate(activationAnimation),
      Paint()..color = sliderTheme.overlayColor!,
    );
  }
}

/// The default shape of a [Slider]'s value indicator.
///
/// ![A slider widget, consisting of 5 divisions and showing the rectangular slider value indicator shape.](https://flutter.github.io/assets-for-api-docs/assets/material/rectangular_slider_value_indicator_shape.png)
///
/// See also:
///
///  * [Slider], which includes a value indicator defined by this shape.
///  * [SliderTheme], which can be used to configure the slider value indicator
///    of all sliders in a widget subtree.
class RectangularSliderValueIndicatorShape extends SliderComponentShape {
  /// Create a slider value indicator that resembles a rectangular tooltip.
  const RectangularSliderValueIndicatorShape();

  static const _RectangularSliderValueIndicatorPathPainter _pathPainter =
      _RectangularSliderValueIndicatorPathPainter();

  @override
  Size getPreferredSize(
    bool isEnabled,
    bool isDiscrete, {
    TextPainter? labelPainter,
    double? textScaleFactor,
  }) {
    assert(labelPainter != null);
    assert(textScaleFactor != null && textScaleFactor >= 0);
    return _pathPainter.getPreferredSize(labelPainter!, textScaleFactor!);
  }

  @override
  void paint(
    PaintingContext context,
    Offset center, {
    required Animation<double> activationAnimation,
    required Animation<double> enableAnimation,
    required bool isDiscrete,
    required TextPainter labelPainter,
    required RenderBox parentBox,
    required SliderThemeData sliderTheme,
    required TextDirection textDirection,
    required double value,
    required double textScaleFactor,
    required Size sizeWithOverflow,
  }) {
    final Canvas canvas = context.canvas;
    final double scale = activationAnimation.value;
    _pathPainter.paint(
      parentBox: parentBox,
      canvas: canvas,
      center: center,
      scale: scale,
      labelPainter: labelPainter,
      textScaleFactor: textScaleFactor,
      sizeWithOverflow: sizeWithOverflow,
      backgroundPaintColor: sliderTheme.valueIndicatorColor!,
      strokePaintColor: sliderTheme.valueIndicatorStrokeColor,
    );
  }
}

/// The default shape of a [RangeSlider]'s value indicators.
///
/// ![A slider widget, consisting of 5 divisions and showing the rectangular range slider value indicator shape.](https://flutter.github.io/assets-for-api-docs/assets/material/rectangular_range_slider_value_indicator_shape.png)
///
/// See also:
///
///  * [RangeSlider], which includes value indicators defined by this shape.
///  * [SliderTheme], which can be used to configure the range slider value
///    indicator of all sliders in a widget subtree.
class RectangularRangeSliderValueIndicatorShape extends RangeSliderValueIndicatorShape {
  /// Create a range slider value indicator that resembles a rectangular tooltip.
  const RectangularRangeSliderValueIndicatorShape();

  static const _RectangularSliderValueIndicatorPathPainter _pathPainter =
      _RectangularSliderValueIndicatorPathPainter();

  @override
  Size getPreferredSize(
    bool isEnabled,
    bool isDiscrete, {
    required TextPainter labelPainter,
    required double textScaleFactor,
  }) {
    assert(textScaleFactor >= 0);
    return _pathPainter.getPreferredSize(labelPainter, textScaleFactor);
  }

  @override
  double getHorizontalShift({
    RenderBox? parentBox,
    Offset? center,
    TextPainter? labelPainter,
    Animation<double>? activationAnimation,
    double? textScaleFactor,
    Size? sizeWithOverflow,
  }) {
    return _pathPainter.getHorizontalShift(
      parentBox: parentBox!,
      center: center!,
      labelPainter: labelPainter!,
      textScaleFactor: textScaleFactor!,
      sizeWithOverflow: sizeWithOverflow!,
      scale: activationAnimation!.value,
    );
  }

  @override
  void paint(
    PaintingContext context,
    Offset center, {
    Animation<double>? activationAnimation,
    Animation<double>? enableAnimation,
    bool? isDiscrete,
    bool? isOnTop,
    TextPainter? labelPainter,
    double? textScaleFactor,
    Size? sizeWithOverflow,
    RenderBox? parentBox,
    SliderThemeData? sliderTheme,
    TextDirection? textDirection,
    double? value,
    Thumb? thumb,
  }) {
    final Canvas canvas = context.canvas;
    final double scale = activationAnimation!.value;
    _pathPainter.paint(
      parentBox: parentBox!,
      canvas: canvas,
      center: center,
      scale: scale,
      labelPainter: labelPainter!,
      textScaleFactor: textScaleFactor!,
      sizeWithOverflow: sizeWithOverflow!,
      backgroundPaintColor: sliderTheme!.valueIndicatorColor!,
      strokePaintColor:
          isOnTop!
              ? sliderTheme.overlappingShapeStrokeColor
              : sliderTheme.valueIndicatorStrokeColor,
    );
  }
}

class _RectangularSliderValueIndicatorPathPainter {
  const _RectangularSliderValueIndicatorPathPainter();

  static const double _triangleHeight = 8.0;
  static const double _labelPadding = 16.0;
  static const double _preferredHeight = 32.0;
  static const double _minLabelWidth = 16.0;
  static const double _bottomTipYOffset = 14.0;
  static const double _preferredHalfHeight = _preferredHeight / 2;
  static const double _upperRectRadius = 4;

  Size getPreferredSize(TextPainter labelPainter, double textScaleFactor) {
    return Size(
      _upperRectangleWidth(labelPainter, 1, textScaleFactor),
      labelPainter.height + _labelPadding,
    );
  }

  double getHorizontalShift({
    required RenderBox parentBox,
    required Offset center,
    required TextPainter labelPainter,
    required double textScaleFactor,
    required Size sizeWithOverflow,
    required double scale,
  }) {
    assert(!sizeWithOverflow.isEmpty);

    const double edgePadding = 8.0;
    final double rectangleWidth = _upperRectangleWidth(labelPainter, scale, textScaleFactor);

    /// Value indicator draws on the Overlay and by using the global Offset
    /// we are making sure we use the bounds of the Overlay instead of the Slider.
    final Offset globalCenter = parentBox.localToGlobal(center);

    // The rectangle must be shifted towards the center so that it minimizes the
    // chance of it rendering outside the bounds of the render box. If the shift
    // is negative, then the lobe is shifted from right to left, and if it is
    // positive, then the lobe is shifted from left to right.
    final double overflowLeft = math.max(0, rectangleWidth / 2 - globalCenter.dx + edgePadding);
    final double overflowRight = math.max(
      0,
      rectangleWidth / 2 - (sizeWithOverflow.width - globalCenter.dx - edgePadding),
    );

    if (rectangleWidth < sizeWithOverflow.width) {
      return overflowLeft - overflowRight;
    } else if (overflowLeft - overflowRight > 0) {
      return overflowLeft - (edgePadding * textScaleFactor);
    } else {
      return -overflowRight + (edgePadding * textScaleFactor);
    }
  }

  double _upperRectangleWidth(TextPainter labelPainter, double scale, double textScaleFactor) {
    final double unscaledWidth =
        math.max(_minLabelWidth * textScaleFactor, labelPainter.width) + _labelPadding * 2;
    return unscaledWidth * scale;
  }

  void paint({
    required RenderBox parentBox,
    required Canvas canvas,
    required Offset center,
    required double scale,
    required TextPainter labelPainter,
    required double textScaleFactor,
    required Size sizeWithOverflow,
    required Color backgroundPaintColor,
    Color? strokePaintColor,
  }) {
    if (scale == 0.0) {
      // Zero scale essentially means "do not draw anything", so it's safe to just return.
      return;
    }
    assert(!sizeWithOverflow.isEmpty);

    final double rectangleWidth = _upperRectangleWidth(labelPainter, scale, textScaleFactor);
    final double horizontalShift = getHorizontalShift(
      parentBox: parentBox,
      center: center,
      labelPainter: labelPainter,
      textScaleFactor: textScaleFactor,
      sizeWithOverflow: sizeWithOverflow,
      scale: scale,
    );

    final double rectHeight = labelPainter.height + _labelPadding;
    final Rect upperRect = Rect.fromLTWH(
      -rectangleWidth / 2 + horizontalShift,
      -_triangleHeight - rectHeight,
      rectangleWidth,
      rectHeight,
    );

    final Path trianglePath =
        Path()
          ..lineTo(-_triangleHeight, -_triangleHeight)
          ..lineTo(_triangleHeight, -_triangleHeight)
          ..close();
    final Paint fillPaint = Paint()..color = backgroundPaintColor;
    final RRect upperRRect = RRect.fromRectAndRadius(
      upperRect,
      const Radius.circular(_upperRectRadius),
    );
    trianglePath.addRRect(upperRRect);

    canvas.save();
    // Prepare the canvas for the base of the tooltip, which is relative to the
    // center of the thumb.
    canvas.translate(center.dx, center.dy - _bottomTipYOffset);
    canvas.scale(scale, scale);
    if (strokePaintColor != null) {
      final Paint strokePaint =
          Paint()
            ..color = strokePaintColor
            ..strokeWidth = 1.0
            ..style = PaintingStyle.stroke;
      canvas.drawPath(trianglePath, strokePaint);
    }
    canvas.drawPath(trianglePath, fillPaint);

    // The label text is centered within the value indicator.
    final double bottomTipToUpperRectTranslateY = -_preferredHalfHeight / 2 - upperRect.height;
    canvas.translate(0, bottomTipToUpperRectTranslateY);
    final Offset boxCenter = Offset(horizontalShift, upperRect.height / 2);
    final Offset halfLabelPainterOffset = Offset(labelPainter.width / 2, labelPainter.height / 2);
    final Offset labelOffset = boxCenter - halfLabelPainterOffset;
    labelPainter.paint(canvas, labelOffset);
    canvas.restore();
  }
}

/// A variant shape of a [Slider]'s value indicator . The value indicator is in
/// the shape of an upside-down pear.
///
/// ![A slider widget, consisting of 5 divisions and showing the paddle slider value indicator shape.](https://flutter.github.io/assets-for-api-docs/assets/material/paddle_slider_value_indicator_shape.png)
///
/// See also:
///
///  * [Slider], which includes a value indicator defined by this shape.
///  * [SliderTheme], which can be used to configure the slider value indicator
///    of all sliders in a widget subtree.
class PaddleSliderValueIndicatorShape extends SliderComponentShape {
  /// Create a slider value indicator in the shape of an upside-down pear.
  const PaddleSliderValueIndicatorShape();

  static const _PaddleSliderValueIndicatorPathPainter _pathPainter =
      _PaddleSliderValueIndicatorPathPainter();

  @override
  Size getPreferredSize(
    bool isEnabled,
    bool isDiscrete, {
    TextPainter? labelPainter,
    double? textScaleFactor,
  }) {
    assert(labelPainter != null);
    assert(textScaleFactor != null && textScaleFactor >= 0);
    return _pathPainter.getPreferredSize(labelPainter!, textScaleFactor!);
  }

  @override
  void paint(
    PaintingContext context,
    Offset center, {
    required Animation<double> activationAnimation,
    required Animation<double> enableAnimation,
    required bool isDiscrete,
    required TextPainter labelPainter,
    required RenderBox parentBox,
    required SliderThemeData sliderTheme,
    required TextDirection textDirection,
    required double value,
    required double textScaleFactor,
    required Size sizeWithOverflow,
  }) {
    assert(!sizeWithOverflow.isEmpty);
    final ColorTween enableColor = ColorTween(
      begin: sliderTheme.disabledThumbColor,
      end: sliderTheme.valueIndicatorColor,
    );
    _pathPainter.paint(
      context.canvas,
      center,
      Paint()..color = enableColor.evaluate(enableAnimation)!,
      activationAnimation.value,
      labelPainter,
      textScaleFactor,
      sizeWithOverflow,
      sliderTheme.valueIndicatorStrokeColor,
    );
  }
}

/// A variant shape of a [RangeSlider]'s value indicators. The value indicator
/// is in the shape of an upside-down pear.
///
/// ![A slider widget, consisting of 5 divisions and showing the paddle range slider value indicator shape.](https://flutter.github.io/assets-for-api-docs/assets/material/paddle_range_slider_value_indicator_shape.png)
///
/// See also:
///
///  * [RangeSlider], which includes value indicators defined by this shape.
///  * [SliderTheme], which can be used to configure the range slider value
///    indicator of all sliders in a widget subtree.
class PaddleRangeSliderValueIndicatorShape extends RangeSliderValueIndicatorShape {
  /// Create a slider value indicator in the shape of an upside-down pear.
  const PaddleRangeSliderValueIndicatorShape();

  static const _PaddleSliderValueIndicatorPathPainter _pathPainter =
      _PaddleSliderValueIndicatorPathPainter();

  @override
  Size getPreferredSize(
    bool isEnabled,
    bool isDiscrete, {
    required TextPainter labelPainter,
    required double textScaleFactor,
  }) {
    assert(textScaleFactor >= 0);
    return _pathPainter.getPreferredSize(labelPainter, textScaleFactor);
  }

  @override
  double getHorizontalShift({
    RenderBox? parentBox,
    Offset? center,
    TextPainter? labelPainter,
    Animation<double>? activationAnimation,
    double? textScaleFactor,
    Size? sizeWithOverflow,
  }) {
    return _pathPainter.getHorizontalShift(
      center: center!,
      labelPainter: labelPainter!,
      scale: activationAnimation!.value,
      textScaleFactor: textScaleFactor!,
      sizeWithOverflow: sizeWithOverflow!,
    );
  }

  @override
  void paint(
    PaintingContext context,
    Offset center, {
    required Animation<double> activationAnimation,
    required Animation<double> enableAnimation,
    bool? isDiscrete,
    bool isOnTop = false,
    required TextPainter labelPainter,
    required RenderBox parentBox,
    required SliderThemeData sliderTheme,
    TextDirection? textDirection,
    Thumb? thumb,
    double? value,
    double? textScaleFactor,
    Size? sizeWithOverflow,
  }) {
    assert(!sizeWithOverflow!.isEmpty);
    final ColorTween enableColor = ColorTween(
      begin: sliderTheme.disabledThumbColor,
      end: sliderTheme.valueIndicatorColor,
    );
    // Add a stroke of 1dp around the top paddle.
    _pathPainter.paint(
      context.canvas,
      center,
      Paint()..color = enableColor.evaluate(enableAnimation)!,
      activationAnimation.value,
      labelPainter,
      textScaleFactor!,
      sizeWithOverflow!,
      isOnTop ? sliderTheme.overlappingShapeStrokeColor : sliderTheme.valueIndicatorStrokeColor,
    );
  }
}

class _PaddleSliderValueIndicatorPathPainter {
  const _PaddleSliderValueIndicatorPathPainter();

  // These constants define the shape of the default value indicator.
  // The value indicator changes shape based on the size of
  // the label: The top lobe spreads horizontally, and the
  // top arc on the neck moves down to keep it merging smoothly
  // with the top lobe as it expands.

  // Radius of the top lobe of the value indicator.
  static const double _topLobeRadius = 16.0;
  static const double _minLabelWidth = 16.0;
  // Radius of the bottom lobe of the value indicator.
  static const double _bottomLobeRadius = 10.0;
  static const double _labelPadding = 8.0;
  static const double _distanceBetweenTopBottomCenters = 40.0;
  static const double _middleNeckWidth = 3.0;
  static const double _bottomNeckRadius = 4.5;
  // The base of the triangle between the top lobe center and the centers of
  // the two top neck arcs.
  static const double _neckTriangleBase = _topNeckRadius + _middleNeckWidth / 2;
  static const double _rightBottomNeckCenterX = _middleNeckWidth / 2 + _bottomNeckRadius;
  static const double _rightBottomNeckAngleStart = math.pi;
  static const Offset _topLobeCenter = Offset(0.0, -_distanceBetweenTopBottomCenters);
  static const double _topNeckRadius = 13.0;
  // The length of the hypotenuse of the triangle formed by the center
  // of the left top lobe arc and the center of the top left neck arc.
  // Used to calculate the position of the center of the arc.
  static const double _neckTriangleHypotenuse = _topLobeRadius + _topNeckRadius;
  // Some convenience values to help readability.
  static const double _twoSeventyDegrees = 3.0 * math.pi / 2.0;
  static const double _ninetyDegrees = math.pi / 2.0;
  static const double _thirtyDegrees = math.pi / 6.0;
  static const double _preferredHeight =
      _distanceBetweenTopBottomCenters + _topLobeRadius + _bottomLobeRadius;
  // Set to true if you want a rectangle to be drawn around the label bubble.
  // This helps with building tests that check that the label draws in the right
  // place (because it prints the rect in the failed test output). It should not
  // be checked in while set to "true".
  static const bool _debuggingLabelLocation = false;

  Size getPreferredSize(TextPainter labelPainter, double textScaleFactor) {
    assert(textScaleFactor >= 0);
    final double width =
        math.max(_minLabelWidth * textScaleFactor, labelPainter.width) +
        _labelPadding * 2 * textScaleFactor;
    return Size(width, _preferredHeight * textScaleFactor);
  }

  // Adds an arc to the path that has the attributes passed in. This is
  // a convenience to make adding arcs have less boilerplate.
  static void _addArc(Path path, Offset center, double radius, double startAngle, double endAngle) {
    assert(center.isFinite);
    final Rect arcRect = Rect.fromCircle(center: center, radius: radius);
    path.arcTo(arcRect, startAngle, endAngle - startAngle, false);
  }

  double getHorizontalShift({
    required Offset center,
    required TextPainter labelPainter,
    required double scale,
    required double textScaleFactor,
    required Size sizeWithOverflow,
  }) {
    assert(!sizeWithOverflow.isEmpty);
    final double inverseTextScale = textScaleFactor != 0 ? 1.0 / textScaleFactor : 0.0;
    final double labelHalfWidth = labelPainter.width / 2.0;
    final double halfWidthNeeded = math.max(
      0.0,
      inverseTextScale * labelHalfWidth - (_topLobeRadius - _labelPadding),
    );
    final double shift = _getIdealOffset(
      halfWidthNeeded,
      textScaleFactor * scale,
      center,
      sizeWithOverflow.width,
    );
    return shift * textScaleFactor;
  }

  // Determines the "best" offset to keep the bubble within the slider. The
  // calling code will bound that with the available movement in the paddle shape.
  double _getIdealOffset(
    double halfWidthNeeded,
    double scale,
    Offset center,
    double widthWithOverflow,
  ) {
    const double edgeMargin = 8.0;
    final Rect topLobeRect = Rect.fromLTWH(
      -_topLobeRadius - halfWidthNeeded,
      -_topLobeRadius - _distanceBetweenTopBottomCenters,
      2.0 * (_topLobeRadius + halfWidthNeeded),
      2.0 * _topLobeRadius,
    );
    // We can just multiply by scale instead of a transform, since we're scaling
    // around (0, 0).
    final Offset topLeft = (topLobeRect.topLeft * scale) + center;
    final Offset bottomRight = (topLobeRect.bottomRight * scale) + center;
    double shift = 0.0;

    if (topLeft.dx < edgeMargin) {
      shift = edgeMargin - topLeft.dx;
    }

    final double endGlobal = widthWithOverflow;
    if (bottomRight.dx > endGlobal - edgeMargin) {
      shift = endGlobal - edgeMargin - bottomRight.dx;
    }

    shift = scale == 0.0 ? 0.0 : shift / scale;
    if (shift < 0.0) {
      // Shifting to the left.
      shift = math.max(shift, -halfWidthNeeded);
    } else {
      // Shifting to the right.
      shift = math.min(shift, halfWidthNeeded);
    }
    return shift;
  }

  void paint(
    Canvas canvas,
    Offset center,
    Paint paint,
    double scale,
    TextPainter labelPainter,
    double textScaleFactor,
    Size sizeWithOverflow,
    Color? strokePaintColor,
  ) {
    if (scale == 0.0) {
      // Zero scale essentially means "do not draw anything", so it's safe to just return. Otherwise,
      // our math below will attempt to divide by zero and send needless NaNs to the engine.
      return;
    }
    assert(!sizeWithOverflow.isEmpty);

    // The entire value indicator should scale with the size of the label,
    // to keep it large enough to encompass the label text.
    final double overallScale = scale * textScaleFactor;
    final double inverseTextScale = textScaleFactor != 0 ? 1.0 / textScaleFactor : 0.0;
    final double labelHalfWidth = labelPainter.width / 2.0;

    canvas.save();
    canvas.translate(center.dx, center.dy);
    canvas.scale(overallScale, overallScale);

    final double bottomNeckTriangleHypotenuse =
        _bottomNeckRadius + _bottomLobeRadius / overallScale;
    final double rightBottomNeckCenterY =
        -math.sqrt(
          math.pow(bottomNeckTriangleHypotenuse, 2) - math.pow(_rightBottomNeckCenterX, 2),
        );
    final double rightBottomNeckAngleEnd =
        math.pi + math.atan(rightBottomNeckCenterY / _rightBottomNeckCenterX);
    final Path path = Path()..moveTo(_middleNeckWidth / 2, rightBottomNeckCenterY);
    _addArc(
      path,
      Offset(_rightBottomNeckCenterX, rightBottomNeckCenterY),
      _bottomNeckRadius,
      _rightBottomNeckAngleStart,
      rightBottomNeckAngleEnd,
    );
    _addArc(
      path,
      Offset.zero,
      _bottomLobeRadius / overallScale,
      rightBottomNeckAngleEnd - math.pi,
      2 * math.pi - rightBottomNeckAngleEnd,
    );
    _addArc(
      path,
      Offset(-_rightBottomNeckCenterX, rightBottomNeckCenterY),
      _bottomNeckRadius,
      math.pi - rightBottomNeckAngleEnd,
      0,
    );

    // This is the needed extra width for the label. It is only positive when
    // the label exceeds the minimum size contained by the round top lobe.
    final double halfWidthNeeded = math.max(
      0.0,
      inverseTextScale * labelHalfWidth - (_topLobeRadius - _labelPadding),
    );

    final double shift = _getIdealOffset(
      halfWidthNeeded,
      overallScale,
      center,
      sizeWithOverflow.width,
    );
    final double leftWidthNeeded = halfWidthNeeded - shift;
    final double rightWidthNeeded = halfWidthNeeded + shift;

    // The parameter that describes how far along the transition from round to
    // stretched we are.
    final double leftAmount = math.max(0.0, math.min(1.0, leftWidthNeeded / _neckTriangleBase));
    final double rightAmount = math.max(0.0, math.min(1.0, rightWidthNeeded / _neckTriangleBase));
    // The angle between the top neck arc's center and the top lobe's center
    // and vertical. The base amount is chosen so that the neck is smooth,
    // even when the lobe is shifted due to its size.
    final double leftTheta = (1.0 - leftAmount) * _thirtyDegrees;
    final double rightTheta = (1.0 - rightAmount) * _thirtyDegrees;
    // The center of the top left neck arc.
    final Offset leftTopNeckCenter = Offset(
      -_neckTriangleBase,
      _topLobeCenter.dy + math.cos(leftTheta) * _neckTriangleHypotenuse,
    );
    final Offset neckRightCenter = Offset(
      _neckTriangleBase,
      _topLobeCenter.dy + math.cos(rightTheta) * _neckTriangleHypotenuse,
    );
    final double leftNeckArcAngle = _ninetyDegrees - leftTheta;
    final double rightNeckArcAngle = math.pi + _ninetyDegrees - rightTheta;
    // The distance between the end of the bottom neck arc and the beginning of
    // the top neck arc. We use this to shrink/expand it based on the scale
    // factor of the value indicator.
    final double neckStretchBaseline = math.max(
      0.0,
      rightBottomNeckCenterY - math.max(leftTopNeckCenter.dy, neckRightCenter.dy),
    );
    final double t = math.pow(inverseTextScale, 3.0) as double;
    final double stretch = clampDouble(neckStretchBaseline * t, 0.0, 10.0 * neckStretchBaseline);
    final Offset neckStretch = Offset(0.0, neckStretchBaseline - stretch);

    assert(
      !_debuggingLabelLocation ||
          () {
            final Offset leftCenter = _topLobeCenter - Offset(leftWidthNeeded, 0.0) + neckStretch;
            final Offset rightCenter = _topLobeCenter + Offset(rightWidthNeeded, 0.0) + neckStretch;
            final Rect valueRect = Rect.fromLTRB(
              leftCenter.dx - _topLobeRadius,
              leftCenter.dy - _topLobeRadius,
              rightCenter.dx + _topLobeRadius,
              rightCenter.dy + _topLobeRadius,
            );
            final Paint outlinePaint =
                Paint()
                  ..color = const Color(0xffff0000)
                  ..style = PaintingStyle.stroke
                  ..strokeWidth = 1.0;
            canvas.drawRect(valueRect, outlinePaint);
            return true;
          }(),
    );

    _addArc(path, leftTopNeckCenter + neckStretch, _topNeckRadius, 0.0, -leftNeckArcAngle);
    _addArc(
      path,
      _topLobeCenter - Offset(leftWidthNeeded, 0.0) + neckStretch,
      _topLobeRadius,
      _ninetyDegrees + leftTheta,
      _twoSeventyDegrees,
    );
    _addArc(
      path,
      _topLobeCenter + Offset(rightWidthNeeded, 0.0) + neckStretch,
      _topLobeRadius,
      _twoSeventyDegrees,
      _twoSeventyDegrees + math.pi - rightTheta,
    );
    _addArc(path, neckRightCenter + neckStretch, _topNeckRadius, rightNeckArcAngle, math.pi);

    if (strokePaintColor != null) {
      final Paint strokePaint =
          Paint()
            ..color = strokePaintColor
            ..strokeWidth = 1.0
            ..style = PaintingStyle.stroke;
      canvas.drawPath(path, strokePaint);
    }

    canvas.drawPath(path, paint);

    // Draw the label.
    canvas.save();
    canvas.translate(shift, -_distanceBetweenTopBottomCenters + neckStretch.dy);
    canvas.scale(inverseTextScale, inverseTextScale);
    labelPainter.paint(canvas, Offset.zero - Offset(labelHalfWidth, labelPainter.height / 2.0));
    canvas.restore();
    canvas.restore();
  }
}

/// A callback that formats a numeric value from a [Slider] or [RangeSlider] widget.
///
/// See also:
///
///  * [Slider.semanticFormatterCallback], which shows an example use case.
///  * [RangeSlider.semanticFormatterCallback], which shows an example use case.
typedef SemanticFormatterCallback = String Function(double value);

/// Decides which thumbs (if any) should be selected.
///
/// The default finds the closest thumb, but if the thumbs are close to each
/// other, it waits for movement defined by [dx] to determine the selected
/// thumb.
///
/// Override [SliderThemeData.thumbSelector] for custom thumb selection.
typedef RangeThumbSelector =
    Thumb? Function(
      TextDirection textDirection,
      RangeValues values,
      double tapValue,
      Size thumbSize,
      Size trackSize,
      double dx,
    );

/// Object for representing range slider thumb values.
///
/// This object is passed into [RangeSlider.values] to set its values, and it
/// is emitted in [RangeSlider.onChanged], [RangeSlider.onChangeStart], and
/// [RangeSlider.onChangeEnd] when the values change.
@immutable
class RangeValues {
  /// Creates pair of start and end values.
  const RangeValues(this.start, this.end);

  /// The value of the start thumb.
  ///
  /// For LTR text direction, the start is the left thumb, and for RTL text
  /// direction, the start is the right thumb.
  final double start;

  /// The value of the end thumb.
  ///
  /// For LTR text direction, the end is the right thumb, and for RTL text
  /// direction, the end is the left thumb.
  final double end;

  @override
  bool operator ==(Object other) {
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is RangeValues && other.start == start && other.end == end;
  }

  @override
  int get hashCode => Object.hash(start, end);

  @override
  String toString() {
    return '${objectRuntimeType(this, 'RangeValues')}($start, $end)';
  }
}

/// Object for setting range slider label values that appear in the value
/// indicator for each thumb.
///
/// Used in combination with [SliderThemeData.showValueIndicator] to display
/// labels above the thumbs.
@immutable
class RangeLabels {
  /// Creates pair of start and end labels.
  const RangeLabels(this.start, this.end);

  /// The label of the start thumb.
  ///
  /// For LTR text direction, the start is the left thumb, and for RTL text
  /// direction, the start is the right thumb.
  final String start;

  /// The label of the end thumb.
  ///
  /// For LTR text direction, the end is the right thumb, and for RTL text
  /// direction, the end is the left thumb.
  final String end;

  @override
  bool operator ==(Object other) {
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is RangeLabels && other.start == start && other.end == end;
  }

  @override
  int get hashCode => Object.hash(start, end);

  @override
  String toString() {
    return '${objectRuntimeType(this, 'RangeLabels')}($start, $end)';
  }
}

void _debugDrawShadow(Canvas canvas, Path path, double elevation) {
  if (elevation > 0.0) {
    canvas.drawPath(
      path,
      Paint()
        ..color = Colors.black
        ..style = PaintingStyle.stroke
        ..strokeWidth = elevation * 2.0,
    );
  }
}

/// The default shape of a Material 3 [Slider]'s value indicator.
///
/// See also:
///
///  * [Slider], which includes a value indicator defined by this shape.
///  * [SliderTheme], which can be used to configure the slider value indicator
///    of all sliders in a widget subtree.
class DropSliderValueIndicatorShape extends SliderComponentShape {
  /// Create a slider value indicator that resembles a drop shape.
  const DropSliderValueIndicatorShape();

  static const _DropSliderValueIndicatorPathPainter _pathPainter =
      _DropSliderValueIndicatorPathPainter();

  @override
  Size getPreferredSize(
    bool isEnabled,
    bool isDiscrete, {
    TextPainter? labelPainter,
    double? textScaleFactor,
  }) {
    assert(labelPainter != null);
    assert(textScaleFactor != null && textScaleFactor >= 0);
    return _pathPainter.getPreferredSize(labelPainter!, textScaleFactor!);
  }

  @override
  void paint(
    PaintingContext context,
    Offset center, {
    required Animation<double> activationAnimation,
    required Animation<double> enableAnimation,
    required bool isDiscrete,
    required TextPainter labelPainter,
    required RenderBox parentBox,
    required SliderThemeData sliderTheme,
    required TextDirection textDirection,
    required double value,
    required double textScaleFactor,
    required Size sizeWithOverflow,
  }) {
    final Canvas canvas = context.canvas;
    final double scale = activationAnimation.value;
    _pathPainter.paint(
      parentBox: parentBox,
      canvas: canvas,
      center: center,
      scale: scale,
      labelPainter: labelPainter,
      textScaleFactor: textScaleFactor,
      sizeWithOverflow: sizeWithOverflow,
      backgroundPaintColor: sliderTheme.valueIndicatorColor!,
      strokePaintColor: sliderTheme.valueIndicatorStrokeColor,
    );
  }
}

class _DropSliderValueIndicatorPathPainter {
  const _DropSliderValueIndicatorPathPainter();

  static const double _triangleHeight = 10.0;
  static const double _labelPadding = 8.0;
  static const double _preferredHeight = 32.0;
  static const double _minLabelWidth = 20.0;
  static const double _minRectHeight = 28.0;
  static const double _rectYOffset = 6.0;
  static const double _bottomTipYOffset = 16.0;
  static const double _preferredHalfHeight = _preferredHeight / 2;
  static const double _upperRectRadius = 4;

  Size getPreferredSize(TextPainter labelPainter, double textScaleFactor) {
    final double width =
        math.max(_minLabelWidth, labelPainter.width) + _labelPadding * 2 * textScaleFactor;
    return Size(width, _preferredHeight * textScaleFactor);
  }

  double getHorizontalShift({
    required RenderBox parentBox,
    required Offset center,
    required TextPainter labelPainter,
    required double textScaleFactor,
    required Size sizeWithOverflow,
    required double scale,
  }) {
    assert(!sizeWithOverflow.isEmpty);

    const double edgePadding = 8.0;
    final double rectangleWidth = _upperRectangleWidth(labelPainter, scale);

    /// Value indicator draws on the Overlay and by using the global Offset
    /// we are making sure we use the bounds of the Overlay instead of the Slider.
    final Offset globalCenter = parentBox.localToGlobal(center);

    // The rectangle must be shifted towards the center so that it minimizes the
    // chance of it rendering outside the bounds of the render box. If the shift
    // is negative, then the lobe is shifted from right to left, and if it is
    // positive, then the lobe is shifted from left to right.
    final double overflowLeft = math.max(0, rectangleWidth / 2 - globalCenter.dx + edgePadding);
    final double overflowRight = math.max(
      0,
      rectangleWidth / 2 - (sizeWithOverflow.width - globalCenter.dx - edgePadding),
    );

    if (rectangleWidth < sizeWithOverflow.width) {
      return overflowLeft - overflowRight;
    } else if (overflowLeft - overflowRight > 0) {
      return overflowLeft - (edgePadding * textScaleFactor);
    } else {
      return -overflowRight + (edgePadding * textScaleFactor);
    }
  }

  double _upperRectangleWidth(TextPainter labelPainter, double scale) {
    final double unscaledWidth = math.max(_minLabelWidth, labelPainter.width) + _labelPadding;
    return unscaledWidth * scale;
  }

  BorderRadius _adjustBorderRadius(Rect rect) {
    const double rectness = 0.0;
    return BorderRadius.lerp(
      BorderRadius.circular(_upperRectRadius),
      BorderRadius.all(Radius.circular(rect.shortestSide / 2.0)),
      1.0 - rectness,
    )!;
  }

  void paint({
    required RenderBox parentBox,
    required Canvas canvas,
    required Offset center,
    required double scale,
    required TextPainter labelPainter,
    required double textScaleFactor,
    required Size sizeWithOverflow,
    required Color backgroundPaintColor,
    Color? strokePaintColor,
  }) {
    if (scale == 0.0) {
      // Zero scale essentially means "do not draw anything", so it's safe to just return.
      return;
    }
    assert(!sizeWithOverflow.isEmpty);
    final double rectangleWidth = _upperRectangleWidth(labelPainter, scale);
    final double horizontalShift = getHorizontalShift(
      parentBox: parentBox,
      center: center,
      labelPainter: labelPainter,
      textScaleFactor: textScaleFactor,
      sizeWithOverflow: sizeWithOverflow,
      scale: scale,
    );
    final Rect upperRect = Rect.fromLTWH(
      -rectangleWidth / 2 + horizontalShift,
      -_rectYOffset - _minRectHeight,
      rectangleWidth,
      _minRectHeight,
    );

    final Paint fillPaint = Paint()..color = backgroundPaintColor;

    canvas.save();
    canvas.translate(center.dx, center.dy - _bottomTipYOffset);
    canvas.scale(scale, scale);

    final BorderRadius adjustedBorderRadius = _adjustBorderRadius(upperRect);
    final RRect borderRect = adjustedBorderRadius
        .resolve(labelPainter.textDirection)
        .toRRect(upperRect);
    final Path trianglePath =
        Path()
          ..lineTo(-_triangleHeight, -_triangleHeight)
          ..lineTo(_triangleHeight, -_triangleHeight)
          ..close();
    trianglePath.addRRect(borderRect);

    if (strokePaintColor != null) {
      final Paint strokePaint =
          Paint()
            ..color = strokePaintColor
            ..strokeWidth = 1.0
            ..style = PaintingStyle.stroke;
      canvas.drawPath(trianglePath, strokePaint);
    }

    canvas.drawPath(trianglePath, fillPaint);

    // The label text is centered within the value indicator.
    final double bottomTipToUpperRectTranslateY = -_preferredHalfHeight / 2 - upperRect.height;
    canvas.translate(0, bottomTipToUpperRectTranslateY);
    final Offset boxCenter = Offset(horizontalShift, upperRect.height / 1.75);
    final Offset halfLabelPainterOffset = Offset(labelPainter.width / 2, labelPainter.height / 2);
    final Offset labelOffset = boxCenter - halfLabelPainterOffset;
    labelPainter.paint(canvas, labelOffset);
    canvas.restore();
  }
}

/// The bar shape of a [Slider]'s thumb.
///
/// When the slider is enabled, the [ColorScheme.primary] color is used for the
/// thumb. When the slider is disabled, the [ColorScheme.onSurface] color with an
/// opacity of 0.38 is used for the thumb.
///
/// The thumb bar shape width is reduced when the thumb is pressed.
///
/// If [SliderThemeData.thumbSize] is null, then the thumb size is 4 pixels for the width
/// and 44 pixels for the height.
///
/// This is the default thumb shape for [Slider]. If [ThemeData.useMaterial3] is false,
/// then the default thumb shape is [RoundSliderThumbShape].
///
/// See also:
///
///  * [Slider], which includes an overlay defined by this shape.
///  * [SliderTheme], which can be used to configure the overlay shape of all
///    sliders in a widget subtree.
class HandleThumbShape extends SliderComponentShape {
  /// Create a slider thumb that draws a bar.
  const HandleThumbShape();

  @override
  Size getPreferredSize(bool isEnabled, bool isDiscrete) {
    return const Size(4.0, 44.0);
  }

  @override
  void paint(
    PaintingContext context,
    Offset center, {
    required Animation<double> activationAnimation,
    required Animation<double> enableAnimation,
    required bool isDiscrete,
    required TextPainter labelPainter,
    required RenderBox parentBox,
    required SliderThemeData sliderTheme,
    required TextDirection textDirection,
    required double value,
    required double textScaleFactor,
    required Size sizeWithOverflow,
  }) {
    assert(sliderTheme.disabledThumbColor != null);
    assert(sliderTheme.thumbColor != null);
    assert(sliderTheme.thumbSize != null);

    final ColorTween colorTween = ColorTween(
      begin: sliderTheme.disabledThumbColor,
      end: sliderTheme.thumbColor,
    );
    final Color color = colorTween.evaluate(enableAnimation)!;

    final Canvas canvas = context.canvas;
    final Size thumbSize =
        sliderTheme.thumbSize!.resolve(<MaterialState>{})!; // This is resolved in the paint method.
    final RRect rrect = RRect.fromRectAndRadius(
      Rect.fromCenter(center: center, width: thumbSize.width, height: thumbSize.height),
      Radius.circular(thumbSize.shortestSide / 2),
    );
    canvas.drawRRect(rrect, Paint()..color = color);
  }
}

/// The gapped shape of a [Slider]'s track.
///
/// The [GappedSliderTrackShape] consists of active and inactive
/// tracks. The active track uses the [SliderThemeData.activeTrackColor] and the
/// inactive track uses the [SliderThemeData.inactiveTrackColor].
///
/// The track shape uses circular corner radius for the edge corners and a corner radius
/// of 2 pixels for the inside corners.
///
/// Between the active and inactive tracks there is a gap of size [SliderThemeData.trackGap].
/// If the [SliderThemeData.thumbShape] is [HandleThumbShape] and the thumb is pressed, the thumb's
/// width is reduced; as a result, the track gap size in [GappedSliderTrackShape]
/// is also reduced.
///
/// If [SliderThemeData.trackGap] is null, then the track gap size defaults to 6 pixels.
///
/// This is the default track shape for [Slider]. If [ThemeData.useMaterial3] is false,
/// then the default track shape is [RoundedRectSliderTrackShape].
///
/// See also:
///
///  * [Slider], which includes an overlay defined by this shape.
///  * [SliderTheme], which can be used to configure the overlay shape of all
///    sliders in a widget subtree.
class GappedSliderTrackShape extends SliderTrackShape with BaseSliderTrackShape {
  /// Create a slider track that draws two rectangles with rounded outer edges.
  const GappedSliderTrackShape();

  @override
  void paint(
    PaintingContext context,
    Offset offset, {
    required RenderBox parentBox,
    required SliderThemeData sliderTheme,
    required Animation<double> enableAnimation,
    required TextDirection textDirection,
    required Offset thumbCenter,
    Offset? secondaryOffset,
    bool isDiscrete = false,
    bool isEnabled = false,
    double additionalActiveTrackHeight = 2,
  }) {
    assert(sliderTheme.disabledActiveTrackColor != null);
    assert(sliderTheme.disabledInactiveTrackColor != null);
    assert(sliderTheme.activeTrackColor != null);
    assert(sliderTheme.inactiveTrackColor != null);
    assert(sliderTheme.thumbShape != null);
    assert(sliderTheme.trackGap != null);
    assert(!sliderTheme.trackGap!.isNegative);
    // If the slider [SliderThemeData.trackHeight] is less than or equal to 0,
    // then it makes no difference whether the track is painted or not,
    // therefore the painting can be a no-op.
    if (sliderTheme.trackHeight == null || sliderTheme.trackHeight! <= 0) {
      return;
    }

    // Assign the track segment paints, which are left: active, right: inactive,
    // but reversed for right to left text.
    final ColorTween activeTrackColorTween = ColorTween(
      begin: sliderTheme.disabledActiveTrackColor,
      end: sliderTheme.activeTrackColor,
    );
    final ColorTween inactiveTrackColorTween = ColorTween(
      begin: sliderTheme.disabledInactiveTrackColor,
      end: sliderTheme.inactiveTrackColor,
    );
    final Paint activePaint = Paint()..color = activeTrackColorTween.evaluate(enableAnimation)!;
    final Paint inactivePaint = Paint()..color = inactiveTrackColorTween.evaluate(enableAnimation)!;
    final Paint leftTrackPaint;
    final Paint rightTrackPaint;
    switch (textDirection) {
      case TextDirection.ltr:
        leftTrackPaint = activePaint;
        rightTrackPaint = inactivePaint;
      case TextDirection.rtl:
        leftTrackPaint = inactivePaint;
        rightTrackPaint = activePaint;
    }

    // Gap, starting from the middle of the thumb.
    final double trackGap = sliderTheme.trackGap!;

    final Rect trackRect = getPreferredRect(
      parentBox: parentBox,
      offset: offset,
      sliderTheme: sliderTheme,
      isEnabled: isEnabled,
      isDiscrete: isDiscrete,
    );

    final Radius trackCornerRadius = Radius.circular(trackRect.shortestSide / 2);
    const Radius trackInsideCornerRadius = Radius.circular(2.0);

    final RRect trackRRect = RRect.fromRectAndCorners(
      trackRect,
      topLeft: trackCornerRadius,
      bottomLeft: trackCornerRadius,
      topRight: trackCornerRadius,
      bottomRight: trackCornerRadius,
    );

    final RRect leftRRect = RRect.fromLTRBAndCorners(
      trackRect.left,
      trackRect.top,
      math.max(trackRect.left, thumbCenter.dx - trackGap),
      trackRect.bottom,
      topLeft: trackCornerRadius,
      bottomLeft: trackCornerRadius,
      topRight: trackInsideCornerRadius,
      bottomRight: trackInsideCornerRadius,
    );

    final RRect rightRRect = RRect.fromLTRBAndCorners(
      thumbCenter.dx + trackGap,
      trackRect.top,
      trackRect.right,
      trackRect.bottom,
      topRight: trackCornerRadius,
      bottomRight: trackCornerRadius,
      topLeft: trackInsideCornerRadius,
      bottomLeft: trackInsideCornerRadius,
    );

    context.canvas
      ..save()
      ..clipRRect(trackRRect);
    final bool drawLeftTrack = thumbCenter.dx > (leftRRect.left + (sliderTheme.trackHeight! / 2));
    final bool drawRightTrack =
        thumbCenter.dx < (rightRRect.right - (sliderTheme.trackHeight! / 2));
    if (drawLeftTrack) {
      context.canvas.drawRRect(leftRRect, leftTrackPaint);
    }
    if (drawRightTrack) {
      context.canvas.drawRRect(rightRRect, rightTrackPaint);
    }

    final bool isLTR = textDirection == TextDirection.ltr;
    final bool showSecondaryTrack =
        (secondaryOffset != null) &&
        switch (isLTR) {
          true => secondaryOffset.dx > thumbCenter.dx + trackGap,
          false => secondaryOffset.dx < thumbCenter.dx - trackGap,
        };

    if (showSecondaryTrack) {
      final ColorTween secondaryTrackColorTween = ColorTween(
        begin: sliderTheme.disabledSecondaryActiveTrackColor,
        end: sliderTheme.secondaryActiveTrackColor,
      );
      final Paint secondaryTrackPaint =
          Paint()..color = secondaryTrackColorTween.evaluate(enableAnimation)!;
      if (isLTR) {
        context.canvas.drawRRect(
          RRect.fromLTRBAndCorners(
            thumbCenter.dx + trackGap,
            trackRect.top,
            secondaryOffset.dx,
            trackRect.bottom,
            topLeft: trackInsideCornerRadius,
            bottomLeft: trackInsideCornerRadius,
            topRight: trackCornerRadius,
            bottomRight: trackCornerRadius,
          ),
          secondaryTrackPaint,
        );
      } else {
        context.canvas.drawRRect(
          RRect.fromLTRBAndCorners(
            secondaryOffset.dx - trackGap,
            trackRect.top,
            thumbCenter.dx,
            trackRect.bottom,
            topLeft: trackInsideCornerRadius,
            bottomLeft: trackInsideCornerRadius,
            topRight: trackCornerRadius,
            bottomRight: trackCornerRadius,
          ),
          secondaryTrackPaint,
        );
      }
    }
    context.canvas.restore();

    const double stopIndicatorRadius = 2.0;
    final double stopIndicatorTrailingSpace = sliderTheme.trackHeight! / 2;
    final Offset stopIndicatorOffset = Offset(
      (textDirection == TextDirection.ltr)
          ? trackRect.centerRight.dx - stopIndicatorTrailingSpace
          : trackRect.centerLeft.dx + stopIndicatorTrailingSpace,
      trackRect.center.dy,
    );

    final bool showStopIndicator =
        (textDirection == TextDirection.ltr)
            ? thumbCenter.dx < stopIndicatorOffset.dx
            : thumbCenter.dx > stopIndicatorOffset.dx;
    if (showStopIndicator && !isDiscrete) {
      final Rect stopIndicatorRect = Rect.fromCircle(
        center: stopIndicatorOffset,
        radius: stopIndicatorRadius,
      );
      context.canvas.drawCircle(stopIndicatorRect.center, stopIndicatorRadius, activePaint);
    }
  }

  @override
  bool get isRounded => true;
}

/// The rounded rectangle shape of a [Slider]'s value indicator.
///
/// If the [SliderThemeData.valueIndicatorColor] is null, then the shape uses the [ColorScheme.inverseSurface]
/// color to draw the value indicator.
///
/// If the [SliderThemeData.valueIndicatorTextStyle] is null, then the indicator label text style
/// defaults to [TextTheme.labelMedium] with the color set to [ColorScheme.onInverseSurface]. If the
/// [ThemeData.useMaterial3] is set to false, then the indicator label text style defaults to
/// [TextTheme.bodyLarge] with the color set to [ColorScheme.onInverseSurface].
///
/// If the [SliderThemeData.valueIndicatorStrokeColor] is provided, then the value indicator is drawn with a
/// stroke border with the color provided.
///
/// This is the default value indicator shape for [Slider]. If [ThemeData.useMaterial3] is false,
/// then the default value indicator shape is [RectangularSliderValueIndicatorShape].
///
/// See also:
///
///  * [Slider], which includes a value indicator defined by this shape.
///  * [SliderTheme], which can be used to configure the slider value indicator
///    of all sliders in a widget subtree.
class RoundedRectSliderValueIndicatorShape extends SliderComponentShape {
  /// Create a slider value indicator that resembles a rounded rectangle.
  const RoundedRectSliderValueIndicatorShape();

  static const _RoundedRectSliderValueIndicatorPathPainter _pathPainter =
      _RoundedRectSliderValueIndicatorPathPainter();

  @override
  Size getPreferredSize(
    bool isEnabled,
    bool isDiscrete, {
    TextPainter? labelPainter,
    double? textScaleFactor,
  }) {
    assert(labelPainter != null);
    assert(textScaleFactor != null && textScaleFactor >= 0);
    return _pathPainter.getPreferredSize(labelPainter!, textScaleFactor!);
  }

  @override
  void paint(
    PaintingContext context,
    Offset center, {
    required Animation<double> activationAnimation,
    required Animation<double> enableAnimation,
    required bool isDiscrete,
    required TextPainter labelPainter,
    required RenderBox parentBox,
    required SliderThemeData sliderTheme,
    required TextDirection textDirection,
    required double value,
    required double textScaleFactor,
    required Size sizeWithOverflow,
  }) {
    final Canvas canvas = context.canvas;
    final double scale = activationAnimation.value;
    _pathPainter.paint(
      parentBox: parentBox,
      canvas: canvas,
      center: center,
      scale: scale,
      labelPainter: labelPainter,
      textScaleFactor: textScaleFactor,
      sizeWithOverflow: sizeWithOverflow,
      backgroundPaintColor: sliderTheme.valueIndicatorColor!,
      strokePaintColor: sliderTheme.valueIndicatorStrokeColor,
    );
  }
}

class _RoundedRectSliderValueIndicatorPathPainter {
  const _RoundedRectSliderValueIndicatorPathPainter();

  static const double _labelPadding = 10.0;
  static const double _preferredHeight = 32.0;
  static const double _minLabelWidth = 16.0;
  static const double _rectYOffset = 10.0;
  static const double _bottomTipYOffset = 16.0;
  static const double _preferredHalfHeight = _preferredHeight / 2;

  Size getPreferredSize(TextPainter labelPainter, double textScaleFactor) {
    final double width =
        math.max(_minLabelWidth, labelPainter.width) + (_labelPadding * 2) * textScaleFactor;
    return Size(width, _preferredHeight * textScaleFactor);
  }

  double getHorizontalShift({
    required RenderBox parentBox,
    required Offset center,
    required TextPainter labelPainter,
    required double textScaleFactor,
    required Size sizeWithOverflow,
    required double scale,
  }) {
    assert(!sizeWithOverflow.isEmpty);

    const double edgePadding = 8.0;
    final double rectangleWidth = _upperRectangleWidth(labelPainter, scale);

    /// Value indicator draws on the Overlay and by using the global Offset
    /// we are making sure we use the bounds of the Overlay instead of the Slider.
    final Offset globalCenter = parentBox.localToGlobal(center);

    // The rectangle must be shifted towards the center so that it minimizes the
    // chance of it rendering outside the bounds of the render box. If the shift
    // is negative, then the lobe is shifted from right to left, and if it is
    // positive, then the lobe is shifted from left to right.
    final double overflowLeft = math.max(0, rectangleWidth / 2 - globalCenter.dx + edgePadding);
    final double overflowRight = math.max(
      0,
      rectangleWidth / 2 - (sizeWithOverflow.width - globalCenter.dx - edgePadding),
    );

    if (rectangleWidth < sizeWithOverflow.width) {
      return overflowLeft - overflowRight;
    } else if (overflowLeft - overflowRight > 0) {
      return overflowLeft - (edgePadding * textScaleFactor);
    } else {
      return -overflowRight + (edgePadding * textScaleFactor);
    }
  }

  double _upperRectangleWidth(TextPainter labelPainter, double scale) {
    final double unscaledWidth = math.max(_minLabelWidth, labelPainter.width) + (_labelPadding * 2);
    return unscaledWidth * scale;
  }

  void paint({
    required RenderBox parentBox,
    required Canvas canvas,
    required Offset center,
    required double scale,
    required TextPainter labelPainter,
    required double textScaleFactor,
    required Size sizeWithOverflow,
    required Color backgroundPaintColor,
    Color? strokePaintColor,
  }) {
    if (scale == 0.0) {
      // Zero scale essentially means "do not draw anything", so it's safe to just return.
      return;
    }
    assert(!sizeWithOverflow.isEmpty);

    final double rectangleWidth = _upperRectangleWidth(labelPainter, scale);
    final double horizontalShift = getHorizontalShift(
      parentBox: parentBox,
      center: center,
      labelPainter: labelPainter,
      textScaleFactor: textScaleFactor,
      sizeWithOverflow: sizeWithOverflow,
      scale: scale,
    );

    final Rect upperRect = Rect.fromLTWH(
      -rectangleWidth / 2 + horizontalShift,
      -_rectYOffset - _preferredHeight,
      rectangleWidth,
      _preferredHeight,
    );

    final Paint fillPaint = Paint()..color = backgroundPaintColor;

    canvas.save();
    // Prepare the canvas for the base of the tooltip, which is relative to the
    // center of the thumb.
    canvas.translate(center.dx, center.dy - _bottomTipYOffset);
    canvas.scale(scale, scale);

    final RRect rrect = RRect.fromRectAndRadius(upperRect, Radius.circular(upperRect.height / 2));
    if (strokePaintColor != null) {
      final Paint strokePaint =
          Paint()
            ..color = strokePaintColor
            ..strokeWidth = 1.0
            ..style = PaintingStyle.stroke;
      canvas.drawRRect(rrect, strokePaint);
    }

    canvas.drawRRect(rrect, fillPaint);

    // The label text is centered within the value indicator.
    final double bottomTipToUpperRectTranslateY = -_preferredHalfHeight / 2 - upperRect.height;
    canvas.translate(0, bottomTipToUpperRectTranslateY);
    final Offset boxCenter = Offset(horizontalShift, upperRect.height / 2.3);
    final Offset halfLabelPainterOffset = Offset(labelPainter.width / 2, labelPainter.height / 2);
    final Offset labelOffset = boxCenter - halfLabelPainterOffset;
    labelPainter.paint(canvas, labelOffset);
    canvas.restore();
  }
}
