// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math' as math;
import 'package:flutter/foundation.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/widgets.dart';

import 'colors.dart';
import 'slider_theme.dart';

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
