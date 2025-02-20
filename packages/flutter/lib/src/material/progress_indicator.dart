// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/semantics.dart';
///
/// @docImport 'refresh_indicator.dart';
library;

import 'dart:math' as math;

import 'package:flutter/cupertino.dart';
import 'package:flutter/foundation.dart';

import 'color_scheme.dart';
import 'material.dart';
import 'progress_indicator_theme.dart';
import 'theme.dart';

const int _kIndeterminateLinearDuration = 1800;
const int _kIndeterminateCircularDuration = 1333 * 2222;

enum _ActivityIndicatorType { material, adaptive }

/// A base class for Material Design progress indicators.
///
/// This widget cannot be instantiated directly. For a linear progress
/// indicator, see [LinearProgressIndicator]. For a circular progress indicator,
/// see [CircularProgressIndicator].
///
/// See also:
///
///  * <https://material.io/components/progress-indicators>
abstract class ProgressIndicator extends StatefulWidget {
  /// Creates a progress indicator.
  ///
  /// {@template flutter.material.ProgressIndicator.ProgressIndicator}
  /// The [value] argument can either be null for an indeterminate
  /// progress indicator, or a non-null value between 0.0 and 1.0 for a
  /// determinate progress indicator.
  ///
  /// ## Accessibility
  ///
  /// The [semanticsLabel] can be used to identify the purpose of this progress
  /// bar for screen reading software. The [semanticsValue] property may be used
  /// for determinate progress indicators to indicate how much progress has been made.
  /// {@endtemplate}
  const ProgressIndicator({
    super.key,
    this.value,
    this.backgroundColor,
    this.color,
    this.valueColor,
    this.semanticsLabel,
    this.semanticsValue,
  });

  /// If non-null, the value of this progress indicator.
  ///
  /// A value of 0.0 means no progress and 1.0 means that progress is complete.
  /// The value will be clamped to be in the range 0.0-1.0.
  ///
  /// If null, this progress indicator is indeterminate, which means the
  /// indicator displays a predetermined animation that does not indicate how
  /// much actual progress is being made.
  final double? value;

  /// The progress indicator's background color.
  ///
  /// It is up to the subclass to implement this in whatever way makes sense
  /// for the given use case. See the subclass documentation for details.
  final Color? backgroundColor;

  /// {@template flutter.progress_indicator.ProgressIndicator.color}
  /// The progress indicator's color.
  ///
  /// This is only used if [ProgressIndicator.valueColor] is null.
  /// If [ProgressIndicator.color] is also null, then the ambient
  /// [ProgressIndicatorThemeData.color] will be used. If that
  /// is null then the current theme's [ColorScheme.primary] will
  /// be used by default.
  /// {@endtemplate}
  final Color? color;

  /// The progress indicator's color as an animated value.
  ///
  /// If null, the progress indicator is rendered with [color]. If that is null,
  /// then it will use the ambient [ProgressIndicatorThemeData.color]. If that
  /// is also null then it defaults to the current theme's [ColorScheme.primary].
  final Animation<Color?>? valueColor;

  /// {@template flutter.progress_indicator.ProgressIndicator.semanticsLabel}
  /// The [SemanticsProperties.label] for this progress indicator.
  ///
  /// This value indicates the purpose of the progress bar, and will be
  /// read out by screen readers to indicate the purpose of this progress
  /// indicator.
  /// {@endtemplate}
  final String? semanticsLabel;

  /// {@template flutter.progress_indicator.ProgressIndicator.semanticsValue}
  /// The [SemanticsProperties.value] for this progress indicator.
  ///
  /// This will be used in conjunction with the [semanticsLabel] by
  /// screen reading software to identify the widget, and is primarily
  /// intended for use with determinate progress indicators to announce
  /// how far along they are.
  ///
  /// For determinate progress indicators, this will be defaulted to
  /// [ProgressIndicator.value] expressed as a percentage, i.e. `0.1` will
  /// become '10%'.
  /// {@endtemplate}
  final String? semanticsValue;

  Color _getValueColor(BuildContext context, {Color? defaultColor}) {
    return valueColor?.value ??
        color ??
        ProgressIndicatorTheme.of(context).color ??
        defaultColor ??
        Theme.of(context).colorScheme.primary;
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(PercentProperty('value', value, showName: false, ifNull: '<indeterminate>'));
  }

  Widget _buildSemanticsWrapper({required BuildContext context, required Widget child}) {
    String? expandedSemanticsValue = semanticsValue;
    if (value != null) {
      expandedSemanticsValue ??= '${(value! * 100).round()}%';
    }
    return Semantics(label: semanticsLabel, value: expandedSemanticsValue, child: child);
  }
}

class _LinearProgressIndicatorPainter extends CustomPainter {
  const _LinearProgressIndicatorPainter({
    required this.trackColor,
    required this.valueColor,
    this.value,
    required this.animationValue,
    required this.textDirection,
    required this.indicatorBorderRadius,
    required this.stopIndicatorColor,
    required this.stopIndicatorRadius,
    required this.trackGap,
  });

  final Color trackColor;
  final Color valueColor;
  final double? value;
  final double animationValue;
  final TextDirection textDirection;
  final BorderRadiusGeometry? indicatorBorderRadius;
  final Color? stopIndicatorColor;
  final double? stopIndicatorRadius;
  final double? trackGap;

  // The indeterminate progress animation displays two lines whose leading (head)
  // and trailing (tail) endpoints are defined by the following four curves.
  static const Curve line1Head = Interval(
    0.0,
    750.0 / _kIndeterminateLinearDuration,
    curve: Cubic(0.2, 0.0, 0.8, 1.0),
  );
  static const Curve line1Tail = Interval(
    333.0 / _kIndeterminateLinearDuration,
    (333.0 + 750.0) / _kIndeterminateLinearDuration,
    curve: Cubic(0.4, 0.0, 1.0, 1.0),
  );
  static const Curve line2Head = Interval(
    1000.0 / _kIndeterminateLinearDuration,
    (1000.0 + 567.0) / _kIndeterminateLinearDuration,
    curve: Cubic(0.0, 0.0, 0.65, 1.0),
  );
  static const Curve line2Tail = Interval(
    1267.0 / _kIndeterminateLinearDuration,
    (1267.0 + 533.0) / _kIndeterminateLinearDuration,
    curve: Cubic(0.10, 0.0, 0.45, 1.0),
  );

  @override
  void paint(Canvas canvas, Size size) {
    final double effectiveTrackGap = switch (value) {
      null || 1.0 => 0.0,
      _ => trackGap ?? 0.0,
    };

    final Rect trackRect;
    if (value != null && effectiveTrackGap > 0) {
      trackRect = switch (textDirection) {
        TextDirection.ltr => Rect.fromLTRB(
          clampDouble(value!, 0.0, 1.0) * size.width + effectiveTrackGap,
          0,
          size.width,
          size.height,
        ),
        TextDirection.rtl => Rect.fromLTRB(
          0,
          0,
          size.width - clampDouble(value!, 0.0, 1.0) * size.width - effectiveTrackGap,
          size.height,
        ),
      };
    } else {
      trackRect = Offset.zero & size;
    }

    // Draw the track.
    final Paint trackPaint = Paint()..color = trackColor;
    if (indicatorBorderRadius != null) {
      final RRect trackRRect = indicatorBorderRadius!.resolve(textDirection).toRRect(trackRect);
      canvas.drawRRect(trackRRect, trackPaint);
    } else {
      canvas.drawRect(trackRect, trackPaint);
    }

    void drawStopIndicator() {
      // Limit the stop indicator radius to the height of the indicator.
      final double radius = math.min(stopIndicatorRadius!, size.height / 2);
      final Paint indicatorPaint = Paint()..color = stopIndicatorColor!;
      final Offset position = switch (textDirection) {
        TextDirection.rtl => Offset(size.height / 2, size.height / 2),
        TextDirection.ltr => Offset(size.width - size.height / 2, size.height / 2),
      };
      canvas.drawCircle(position, radius, indicatorPaint);
    }

    // Draw the stop indicator.
    if (value != null && stopIndicatorRadius != null && stopIndicatorRadius! > 0) {
      drawStopIndicator();
    }

    void drawActiveIndicator(double x, double width) {
      if (width <= 0.0) {
        return;
      }
      final Paint activeIndicatorPaint = Paint()..color = valueColor;
      final double left = switch (textDirection) {
        TextDirection.rtl => size.width - width - x,
        TextDirection.ltr => x,
      };

      final Rect activeRect = Offset(left, 0.0) & Size(width, size.height);
      if (indicatorBorderRadius != null) {
        final RRect activeRRect = indicatorBorderRadius!.resolve(textDirection).toRRect(activeRect);
        canvas.drawRRect(activeRRect, activeIndicatorPaint);
      } else {
        canvas.drawRect(activeRect, activeIndicatorPaint);
      }
    }

    // Draw the active indicator.
    if (value != null) {
      drawActiveIndicator(0.0, clampDouble(value!, 0.0, 1.0) * size.width);
    } else {
      final double x1 = size.width * line1Tail.transform(animationValue);
      final double width1 = size.width * line1Head.transform(animationValue) - x1;

      final double x2 = size.width * line2Tail.transform(animationValue);
      final double width2 = size.width * line2Head.transform(animationValue) - x2;

      drawActiveIndicator(x1, width1);
      drawActiveIndicator(x2, width2);
    }
  }

  @override
  bool shouldRepaint(_LinearProgressIndicatorPainter oldPainter) {
    return oldPainter.trackColor != trackColor ||
        oldPainter.valueColor != valueColor ||
        oldPainter.value != value ||
        oldPainter.animationValue != animationValue ||
        oldPainter.textDirection != textDirection ||
        oldPainter.indicatorBorderRadius != indicatorBorderRadius ||
        oldPainter.stopIndicatorColor != stopIndicatorColor ||
        oldPainter.stopIndicatorRadius != stopIndicatorRadius ||
        oldPainter.trackGap != trackGap;
  }
}

/// A Material Design linear progress indicator, also known as a progress bar.
///
/// {@youtube 560 315 https://www.youtube.com/watch?v=O-rhXZLtpv0}
///
/// A widget that shows progress along a line. There are two kinds of linear
/// progress indicators:
///
///  * _Determinate_. Determinate progress indicators have a specific value at
///    each point in time, and the value should increase monotonically from 0.0
///    to 1.0, at which time the indicator is complete. To create a determinate
///    progress indicator, use a non-null [value] between 0.0 and 1.0.
///  * _Indeterminate_. Indeterminate progress indicators do not have a specific
///    value at each point in time and instead indicate that progress is being
///    made without indicating how much progress remains. To create an
///    indeterminate progress indicator, use a null [value].
///
/// The indicator line is displayed with [valueColor], an animated value. To
/// specify a constant color value use: `AlwaysStoppedAnimation<Color>(color)`.
///
/// The minimum height of the indicator can be specified using [minHeight].
/// The indicator can be made taller by wrapping the widget with a [SizedBox].
///
/// {@tool dartpad}
/// This example showcases determinate and indeterminate [LinearProgressIndicator]s.
/// The [LinearProgressIndicator]s will use the ![updated Material 3 Design appearance](https://m3.material.io/components/progress-indicators/overview)
/// when setting the [LinearProgressIndicator.year2023] flag to false.
///
/// ** See code in examples/api/lib/material/progress_indicator/linear_progress_indicator.0.dart **
/// {@end-tool}
///
/// {@tool dartpad}
/// This sample shows the creation of a [LinearProgressIndicator] with a changing value.
/// When toggling the switch, [LinearProgressIndicator] uses a determinate value.
/// As described in: https://m3.material.io/components/progress-indicators/overview
///
/// ** See code in examples/api/lib/material/progress_indicator/linear_progress_indicator.1.dart **
/// {@end-tool}
///
/// See also:
///
///  * [CircularProgressIndicator], which shows progress along a circular arc.
///  * [RefreshIndicator], which automatically displays a [CircularProgressIndicator]
///    when the underlying vertical scrollable is overscrolled.
///  * <https://material.io/design/components/progress-indicators.html#linear-progress-indicators>
class LinearProgressIndicator extends ProgressIndicator {
  /// Creates a linear progress indicator.
  ///
  /// {@macro flutter.material.ProgressIndicator.ProgressIndicator}
  const LinearProgressIndicator({
    super.key,
    super.value,
    super.backgroundColor,
    super.color,
    super.valueColor,
    this.minHeight,
    super.semanticsLabel,
    super.semanticsValue,
    this.borderRadius,
    this.stopIndicatorColor,
    this.stopIndicatorRadius,
    this.trackGap,
    @Deprecated(
      'Set this flag to false to opt into the 2024 progress indicator appearance. Defaults to true. '
      'In the future, this flag will default to false. Use ProgressIndicatorThemeData to customize individual properties. '
      'This feature was deprecated after v3.26.0-0.1.pre.',
    )
    this.year2023,
  }) : assert(minHeight == null || minHeight > 0);

  /// {@template flutter.material.LinearProgressIndicator.trackColor}
  /// Color of the track being filled by the linear indicator.
  ///
  /// If [LinearProgressIndicator.backgroundColor] is null then the
  /// ambient [ProgressIndicatorThemeData.linearTrackColor] will be used.
  /// If that is null, then the ambient theme's [ColorScheme.background]
  /// will be used to draw the track.
  /// {@endtemplate}
  @override
  Color? get backgroundColor => super.backgroundColor;

  /// {@template flutter.material.LinearProgressIndicator.minHeight}
  /// The minimum height of the line used to draw the linear indicator.
  ///
  /// If [LinearProgressIndicator.minHeight] is null then it will use the
  /// ambient [ProgressIndicatorThemeData.linearMinHeight]. If that is null
  /// it will use 4dp.
  /// {@endtemplate}
  final double? minHeight;

  /// The border radius of both the indicator and the track.
  ///
  /// If null, then the [ProgressIndicatorThemeData.borderRadius] will be used.
  /// If that is also null, then defaults to radius of 2, which produces a
  /// rounded shape with a rounded indicator. If [ThemeData.useMaterial3] is false,
  /// then defaults to [BorderRadius.zero], which produces a rectangular shape
  /// with a rectangular indicator.
  final BorderRadiusGeometry? borderRadius;

  /// The color of the stop indicator.
  ///
  /// If [year2023] is false or [ThemeData.useMaterial3] is false, then no stop
  /// indicator will be drawn.
  ///
  /// If null, then the [ProgressIndicatorThemeData.stopIndicatorColor] will be used.
  /// If that is null, then the [ColorScheme.primary] will be used.
  final Color? stopIndicatorColor;

  /// The radius of the stop indicator.
  ///
  /// If [year2023] is false or [ThemeData.useMaterial3] is false, then no stop
  /// indicator will be drawn.
  ///
  /// Set [stopIndicatorRadius] to 0 to hide the stop indicator.
  ///
  /// If null, then the [ProgressIndicatorThemeData.stopIndicatorRadius] will be used.
  /// If that is null, then defaults to 2.
  final double? stopIndicatorRadius;

  /// The gap between the indicator and the track.
  ///
  /// If [year2023] is false or [ThemeData.useMaterial3] is false, then no track
  /// gap will be drawn.
  ///
  /// Set [trackGap] to 0 to hide the track gap.
  ///
  /// If null, then the [ProgressIndicatorThemeData.trackGap] will be used.
  /// If that is null, then defaults to 4.
  final double? trackGap;

  /// When true, the [LinearProgressIndicator] will use the 2023 Material Design 3
  /// appearance.
  ///
  /// If null, then the [ProgressIndicatorThemeData.year2023] will be used.
  /// If that is null, then defaults to true.
  ///
  /// If this is set to false, the [LinearProgressIndicator] will use the
  /// latest Material Design 3 appearance, which was introduced in December 2023.
  ///
  /// If [ThemeData.useMaterial3] is false, then this property is ignored.
  @Deprecated(
    'Set this flag to false to opt into the 2024 progress indicator appearance. Defaults to true. '
    'In the future, this flag will default to false. Use ProgressIndicatorThemeData to customize individual properties. '
    'This feature was deprecated after v3.27.0-0.1.pre.',
  )
  final bool? year2023;

  @override
  State<LinearProgressIndicator> createState() => _LinearProgressIndicatorState();
}

class _LinearProgressIndicatorState extends State<LinearProgressIndicator>
    with SingleTickerProviderStateMixin {
  late AnimationController _controller;

  @override
  void initState() {
    super.initState();
    _controller = AnimationController(
      duration: const Duration(milliseconds: _kIndeterminateLinearDuration),
      vsync: this,
    );
    if (widget.value == null) {
      _controller.repeat();
    }
  }

  @override
  void didUpdateWidget(LinearProgressIndicator oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (widget.value == null && !_controller.isAnimating) {
      _controller.repeat();
    } else if (widget.value != null && _controller.isAnimating) {
      _controller.stop();
    }
  }

  @override
  void dispose() {
    _controller.dispose();
    super.dispose();
  }

  Widget _buildIndicator(BuildContext context, double animationValue, TextDirection textDirection) {
    final ProgressIndicatorThemeData indicatorTheme = ProgressIndicatorTheme.of(context);
    final bool year2023 = widget.year2023 ?? indicatorTheme.year2023 ?? true;
    final ProgressIndicatorThemeData defaults = switch (Theme.of(context).useMaterial3) {
      true =>
        year2023
            ? _LinearProgressIndicatorDefaultsM3Year2023(context)
            : _LinearProgressIndicatorDefaultsM3(context),
      false => _LinearProgressIndicatorDefaultsM2(context),
    };
    final Color trackColor =
        widget.backgroundColor ?? indicatorTheme.linearTrackColor ?? defaults.linearTrackColor!;
    final double minHeight =
        widget.minHeight ?? indicatorTheme.linearMinHeight ?? defaults.linearMinHeight!;
    final BorderRadiusGeometry? borderRadius =
        widget.borderRadius ?? indicatorTheme.borderRadius ?? defaults.borderRadius;
    final Color? stopIndicatorColor =
        !year2023
            ? widget.stopIndicatorColor ??
                indicatorTheme.stopIndicatorColor ??
                defaults.stopIndicatorColor
            : null;
    final double? stopIndicatorRadius =
        !year2023
            ? widget.stopIndicatorRadius ??
                indicatorTheme.stopIndicatorRadius ??
                defaults.stopIndicatorRadius
            : null;
    final double? trackGap =
        !year2023 ? widget.trackGap ?? indicatorTheme.trackGap ?? defaults.trackGap : null;

    Widget result = ConstrainedBox(
      constraints: BoxConstraints(minWidth: double.infinity, minHeight: minHeight),
      child: CustomPaint(
        painter: _LinearProgressIndicatorPainter(
          trackColor: trackColor,
          valueColor: widget._getValueColor(context, defaultColor: defaults.color),
          value: widget.value, // may be null
          animationValue: animationValue, // ignored if widget.value is not null
          textDirection: textDirection,
          indicatorBorderRadius: borderRadius,
          stopIndicatorColor: stopIndicatorColor,
          stopIndicatorRadius: stopIndicatorRadius,
          trackGap: trackGap,
        ),
      ),
    );

    // Clip is only needed with indeterminate progress indicators
    if (borderRadius != null && widget.value == null) {
      result = ClipRRect(borderRadius: borderRadius, child: result);
    }

    return widget._buildSemanticsWrapper(context: context, child: result);
  }

  @override
  Widget build(BuildContext context) {
    final TextDirection textDirection = Directionality.of(context);

    if (widget.value != null) {
      return _buildIndicator(context, _controller.value, textDirection);
    }

    return AnimatedBuilder(
      animation: _controller.view,
      builder: (BuildContext context, Widget? child) {
        return _buildIndicator(context, _controller.value, textDirection);
      },
    );
  }
}

class _CircularProgressIndicatorPainter extends CustomPainter {
  _CircularProgressIndicatorPainter({
    this.trackColor,
    required this.valueColor,
    required this.value,
    required this.headValue,
    required this.tailValue,
    required this.offsetValue,
    required this.rotationValue,
    required this.strokeWidth,
    required this.strokeAlign,
    this.strokeCap,
    this.trackGap,
    this.year2023 = true,
  }) : arcStart =
           value != null
               ? _startAngle
               : _startAngle +
                   tailValue * 3 / 2 * math.pi +
                   rotationValue * math.pi * 2.0 +
                   offsetValue * 0.5 * math.pi,
       arcSweep =
           value != null
               ? clampDouble(value, 0.0, 1.0) * _sweep
               : math.max(headValue * 3 / 2 * math.pi - tailValue * 3 / 2 * math.pi, _epsilon);

  final Color? trackColor;
  final Color valueColor;
  final double? value;
  final double headValue;
  final double tailValue;
  final double offsetValue;
  final double rotationValue;
  final double strokeWidth;
  final double strokeAlign;
  final double arcStart;
  final double arcSweep;
  final StrokeCap? strokeCap;
  final double? trackGap;
  final bool year2023;

  static const double _twoPi = math.pi * 2.0;
  static const double _epsilon = .001;
  // Canvas.drawArc(r, 0, 2*PI) doesn't draw anything, so just get close.
  static const double _sweep = _twoPi - _epsilon;
  static const double _startAngle = -math.pi / 2.0;

  @override
  void paint(Canvas canvas, Size size) {
    final Paint paint =
        Paint()
          ..color = valueColor
          ..strokeWidth = strokeWidth
          ..style = PaintingStyle.stroke;

    // Use the negative operator as intended to keep the exposed constant value
    // as users are already familiar with.
    final double strokeOffset = strokeWidth / 2 * -strokeAlign;
    final Offset arcBaseOffset = Offset(strokeOffset, strokeOffset);
    final Size arcActualSize = Size(size.width - strokeOffset * 2, size.height - strokeOffset * 2);
    final bool hasGap = trackGap != null && trackGap! > 0;

    if (trackColor != null) {
      final Paint backgroundPaint =
          Paint()
            ..color = trackColor!
            ..strokeWidth = strokeWidth
            ..strokeCap = strokeCap ?? StrokeCap.round
            ..style = PaintingStyle.stroke;
      // If hasGap is true, draw the background arc with a gap.
      if (hasGap && value != null && value! > _epsilon) {
        final double arcRadius = arcActualSize.shortestSide / 2;
        final double strokeRadius = strokeWidth / arcRadius;
        final double gapRadius = trackGap! / arcRadius;
        final double startGap = strokeRadius + gapRadius;
        final double endGap = value! < _epsilon ? startGap : startGap * 2;
        final double startSweep = (-math.pi / 2.0) + startGap;
        final double endSweep = math.max(
          0.0,
          _twoPi - clampDouble(value!, 0.0, 1.0) * _twoPi - endGap,
        );
        // Flip the canvas for the background arc.
        canvas.save();
        canvas.scale(-1, 1);
        canvas.translate(-size.width, 0);
        canvas.drawArc(arcBaseOffset & arcActualSize, startSweep, endSweep, false, backgroundPaint);
        // Restore the canvas to draw the foreground arc.
        canvas.restore();
      } else {
        canvas.drawArc(arcBaseOffset & arcActualSize, 0, _sweep, false, backgroundPaint);
      }
    }

    if (year2023) {
      if (value == null && strokeCap == null) {
        // Indeterminate
        paint.strokeCap = StrokeCap.square;
      } else {
        // Butt when determinate (value != null) && strokeCap == null;
        paint.strokeCap = strokeCap ?? StrokeCap.butt;
      }
    } else {
      paint.strokeCap = strokeCap ?? StrokeCap.round;
    }

    canvas.drawArc(arcBaseOffset & arcActualSize, arcStart, arcSweep, false, paint);
  }

  @override
  bool shouldRepaint(_CircularProgressIndicatorPainter oldPainter) {
    return oldPainter.trackColor != trackColor ||
        oldPainter.valueColor != valueColor ||
        oldPainter.value != value ||
        oldPainter.headValue != headValue ||
        oldPainter.tailValue != tailValue ||
        oldPainter.offsetValue != offsetValue ||
        oldPainter.rotationValue != rotationValue ||
        oldPainter.strokeWidth != strokeWidth ||
        oldPainter.strokeAlign != strokeAlign ||
        oldPainter.strokeCap != strokeCap ||
        oldPainter.trackGap != trackGap ||
        oldPainter.year2023 != year2023;
  }
}

/// A Material Design circular progress indicator, which spins to indicate that
/// the application is busy.
///
/// {@youtube 560 315 https://www.youtube.com/watch?v=O-rhXZLtpv0}
///
/// A widget that shows progress along a circle. There are two kinds of circular
/// progress indicators:
///
///  * _Determinate_. Determinate progress indicators have a specific value at
///    each point in time, and the value should increase monotonically from 0.0
///    to 1.0, at which time the indicator is complete. To create a determinate
///    progress indicator, use a non-null [value] between 0.0 and 1.0.
///  * _Indeterminate_. Indeterminate progress indicators do not have a specific
///    value at each point in time and instead indicate that progress is being
///    made without indicating how much progress remains. To create an
///    indeterminate progress indicator, use a null [value].
///
/// The indicator arc is displayed with [valueColor], an animated value. To
/// specify a constant color use: `AlwaysStoppedAnimation<Color>(color)`.
///
/// {@tool dartpad}
/// This example showcases determinate and indeterminate [CircularProgressIndicator]s.
/// The [CircularProgressIndicator]s will use the ![updated Material 3 Design appearance](https://m3.material.io/components/progress-indicators/overview)
/// when setting the [CircularProgressIndicator.year2023] flag to false.
///
/// ** See code in examples/api/lib/material/progress_indicator/circular_progress_indicator.0.dart **
/// {@end-tool}
///
/// {@tool dartpad}
/// This sample shows the creation of a [CircularProgressIndicator] with a changing value.
/// When toggling the switch, [CircularProgressIndicator] uses a determinate value.
/// As described in: https://m3.material.io/components/progress-indicators/overview
///
/// ** See code in examples/api/lib/material/progress_indicator/circular_progress_indicator.1.dart **
/// {@end-tool}
///
/// See also:
///
///  * [LinearProgressIndicator], which displays progress along a line.
///  * [RefreshIndicator], which automatically displays a [CircularProgressIndicator]
///    when the underlying vertical scrollable is overscrolled.
///  * <https://material.io/design/components/progress-indicators.html#circular-progress-indicators>
class CircularProgressIndicator extends ProgressIndicator {
  /// Creates a circular progress indicator.
  ///
  /// {@macro flutter.material.ProgressIndicator.ProgressIndicator}
  const CircularProgressIndicator({
    super.key,
    super.value,
    super.backgroundColor,
    super.color,
    super.valueColor,
    this.strokeWidth,
    this.strokeAlign,
    super.semanticsLabel,
    super.semanticsValue,
    this.strokeCap,
    this.constraints,
    this.trackGap,
    @Deprecated(
      'Set this flag to false to opt into the 2024 progress indicator appearance. Defaults to true. '
      'In the future, this flag will default to false. Use ProgressIndicatorThemeData to customize individual properties. '
      'This feature was deprecated after v3.27.0-0.1.pre.',
    )
    this.year2023,
    this.padding,
  }) : _indicatorType = _ActivityIndicatorType.material;

  /// Creates an adaptive progress indicator that is a
  /// [CupertinoActivityIndicator] on [TargetPlatform.iOS] &
  /// [TargetPlatform.macOS] and a [CircularProgressIndicator] in material
  /// theme/non-Apple platforms.
  ///
  /// The [valueColor], [strokeWidth], [strokeAlign], [strokeCap],
  /// [semanticsLabel], [semanticsValue], [trackGap], [year2023] will be
  /// ignored on iOS & macOS.
  ///
  /// {@macro flutter.material.ProgressIndicator.ProgressIndicator}
  const CircularProgressIndicator.adaptive({
    super.key,
    super.value,
    super.backgroundColor,
    super.valueColor,
    this.strokeWidth = 4.0,
    super.semanticsLabel,
    super.semanticsValue,
    this.strokeCap,
    this.strokeAlign,
    this.constraints,
    this.trackGap,
    @Deprecated(
      'Set this flag to false to opt into the 2024 progress indicator appearance. Defaults to true. '
      'In the future, this flag will default to false. Use ProgressIndicatorThemeData to customize individual properties. '
      'This feature was deprecated after v3.27.0-0.2.pre.',
    )
    this.year2023,
    this.padding,
  }) : _indicatorType = _ActivityIndicatorType.adaptive;

  final _ActivityIndicatorType _indicatorType;

  /// {@template flutter.material.CircularProgressIndicator.trackColor}
  /// Color of the circular track being filled by the circular indicator.
  ///
  /// If [CircularProgressIndicator.backgroundColor] is null then the
  /// ambient [ProgressIndicatorThemeData.circularTrackColor] will be used.
  /// If that is null, then the track will not be painted.
  /// {@endtemplate}
  @override
  Color? get backgroundColor => super.backgroundColor;

  /// The width of the line used to draw the circle.
  final double? strokeWidth;

  /// The relative position of the stroke on a [CircularProgressIndicator].
  ///
  /// Values typically range from -1.0 ([strokeAlignInside], inside stroke)
  /// to 1.0 ([strokeAlignOutside], outside stroke),
  /// without any bound constraints (e.g., a value of -2.0 is not typical, but allowed).
  /// A value of 0 ([strokeAlignCenter]) will center the border
  /// on the edge of the widget.
  ///
  /// If [year2023] is true, then the default value is [strokeAlignCenter].
  /// Otherwise, the default value is [strokeAlignInside].
  final double? strokeAlign;

  /// The progress indicator's line ending.
  ///
  /// This determines the shape of the stroke ends of the progress indicator.
  /// By default, [strokeCap] is null.
  /// When [value] is null (indeterminate), the stroke ends are set to
  /// [StrokeCap.square]. When [value] is not null, the stroke
  /// ends are set to [StrokeCap.butt].
  ///
  /// Setting [strokeCap] to [StrokeCap.round] will result in a rounded end.
  /// Setting [strokeCap] to [StrokeCap.butt] with [value] == null will result
  /// in a slightly different indeterminate animation; the indicator completely
  /// disappears and reappears on its minimum value.
  /// Setting [strokeCap] to [StrokeCap.square] with [value] != null will
  /// result in a different display of [value]. The indicator will start
  /// drawing from slightly less than the start, and end slightly after
  /// the end. This will produce an alternative result, as the
  /// default behavior, for example, that a [value] of 0.5 starts at 90 degrees
  /// and ends at 270 degrees. With [StrokeCap.square], it could start 85
  /// degrees and end at 275 degrees.
  final StrokeCap? strokeCap;

  /// Defines minimum and maximum sizes for a [CircularProgressIndicator].
  ///
  /// If null, then the [ProgressIndicatorThemeData.constraints] will be used.
  /// Otherwise, defaults to a minimum width and height of 36 pixels.
  final BoxConstraints? constraints;

  /// The gap between the active indicator and the background track.
  ///
  /// If [year2023] is false or [ThemeData.useMaterial3] is false, then no track
  /// gap will be drawn.
  ///
  /// Set [trackGap] to 0 to hide the track gap.
  ///
  /// If null, then the [ProgressIndicatorThemeData.trackGap] will be used.
  /// If that is null, then defaults to 4.
  final double? trackGap;

  /// When true, the [CircularProgressIndicator] will use the 2023 Material Design 3
  /// appearance.
  ///
  /// If null, then the [ProgressIndicatorThemeData.year2023] will be used.
  /// If that is null, then defaults to true.
  ///
  /// If this is set to false, the [CircularProgressIndicator] will use the
  /// latest Material Design 3 appearance, which was introduced in December 2023.
  ///
  /// If [ThemeData.useMaterial3] is false, then this property is ignored.
  @Deprecated(
    'Set this flag to false to opt into the 2024 progress indicator appearance. Defaults to true. '
    'In the future, this flag will default to false. Use ProgressIndicatorThemeData to customize individual properties. '
    'This feature was deprecated after v3.27.0-0.2.pre.',
  )
  final bool? year2023;

  /// The padding around the indicator track.
  ///
  /// If null, then the [ProgressIndicatorThemeData.circularTrackPadding] will be
  /// used. If that is null and [year2023] is false, then defaults to `EdgeInsets.all(4.0)`
  /// padding. Otherwise, defaults to zero padding.
  final EdgeInsetsGeometry? padding;

  /// The indicator stroke is drawn fully inside of the indicator path.
  ///
  /// This is a constant for use with [strokeAlign].
  static const double strokeAlignInside = -1.0;

  /// The indicator stroke is drawn on the center of the indicator path,
  /// with half of the [strokeWidth] on the inside, and the other half
  /// on the outside of the path.
  ///
  /// This is a constant for use with [strokeAlign].
  ///
  /// This is the default value for [strokeAlign].
  static const double strokeAlignCenter = 0.0;

  /// The indicator stroke is drawn on the outside of the indicator path.
  ///
  /// This is a constant for use with [strokeAlign].
  static const double strokeAlignOutside = 1.0;

  @override
  State<CircularProgressIndicator> createState() => _CircularProgressIndicatorState();
}

class _CircularProgressIndicatorState extends State<CircularProgressIndicator>
    with SingleTickerProviderStateMixin {
  static const int _pathCount = _kIndeterminateCircularDuration ~/ 1333;
  static const int _rotationCount = _kIndeterminateCircularDuration ~/ 2222;

  static final Animatable<double> _strokeHeadTween = CurveTween(
    curve: const Interval(0.0, 0.5, curve: Curves.fastOutSlowIn),
  ).chain(CurveTween(curve: const SawTooth(_pathCount)));
  static final Animatable<double> _strokeTailTween = CurveTween(
    curve: const Interval(0.5, 1.0, curve: Curves.fastOutSlowIn),
  ).chain(CurveTween(curve: const SawTooth(_pathCount)));
  static final Animatable<double> _offsetTween = CurveTween(curve: const SawTooth(_pathCount));
  static final Animatable<double> _rotationTween = CurveTween(
    curve: const SawTooth(_rotationCount),
  );

  late AnimationController _controller;

  @override
  void initState() {
    super.initState();
    _controller = AnimationController(
      duration: const Duration(milliseconds: _kIndeterminateCircularDuration),
      vsync: this,
    );
    if (widget.value == null) {
      _controller.repeat();
    }
  }

  @override
  void didUpdateWidget(CircularProgressIndicator oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (widget.value == null && !_controller.isAnimating) {
      _controller.repeat();
    } else if (widget.value != null && _controller.isAnimating) {
      _controller.stop();
    }
  }

  @override
  void dispose() {
    _controller.dispose();
    super.dispose();
  }

  Widget _buildCupertinoIndicator(BuildContext context) {
    final Color? tickColor = widget.backgroundColor;
    final double? value = widget.value;
    if (value == null) {
      return CupertinoActivityIndicator(key: widget.key, color: tickColor);
    }
    return CupertinoActivityIndicator.partiallyRevealed(
      key: widget.key,
      color: tickColor,
      progress: value,
    );
  }

  Widget _buildMaterialIndicator(
    BuildContext context,
    double headValue,
    double tailValue,
    double offsetValue,
    double rotationValue,
  ) {
    final ProgressIndicatorThemeData indicatorTheme = ProgressIndicatorTheme.of(context);
    final bool year2023 = widget.year2023 ?? indicatorTheme.year2023 ?? true;
    final ProgressIndicatorThemeData defaults = switch (Theme.of(context).useMaterial3) {
      true =>
        year2023
            ? _CircularProgressIndicatorDefaultsM3Year2023(
              context,
              indeterminate: widget.value == null,
            )
            : _CircularProgressIndicatorDefaultsM3(context, indeterminate: widget.value == null),
      false => _CircularProgressIndicatorDefaultsM2(context, indeterminate: widget.value == null),
    };
    final Color? trackColor =
        widget.backgroundColor ?? indicatorTheme.circularTrackColor ?? defaults.circularTrackColor;
    final double strokeWidth =
        widget.strokeWidth ?? indicatorTheme.strokeWidth ?? defaults.strokeWidth!;
    final double strokeAlign =
        widget.strokeAlign ?? indicatorTheme.strokeAlign ?? defaults.strokeAlign!;
    final StrokeCap? strokeCap = widget.strokeCap ?? indicatorTheme.strokeCap;
    final BoxConstraints constraints =
        widget.constraints ?? indicatorTheme.constraints ?? defaults.constraints!;
    final double? trackGap =
        year2023 ? null : widget.trackGap ?? indicatorTheme.trackGap ?? defaults.trackGap;
    final EdgeInsetsGeometry? effectivePadding =
        widget.padding ?? indicatorTheme.circularTrackPadding ?? defaults.circularTrackPadding;

    Widget result = ConstrainedBox(
      constraints: constraints,
      child: CustomPaint(
        painter: _CircularProgressIndicatorPainter(
          trackColor: trackColor,
          valueColor: widget._getValueColor(context, defaultColor: defaults.color),
          value: widget.value, // may be null
          headValue: headValue, // remaining arguments are ignored if widget.value is not null
          tailValue: tailValue,
          offsetValue: offsetValue,
          rotationValue: rotationValue,
          strokeWidth: strokeWidth,
          strokeAlign: strokeAlign,
          strokeCap: strokeCap,
          trackGap: trackGap,
          year2023: year2023,
        ),
      ),
    );

    if (effectivePadding != null) {
      result = Padding(padding: effectivePadding, child: result);
    }

    return widget._buildSemanticsWrapper(context: context, child: result);
  }

  Widget _buildAnimation() {
    return AnimatedBuilder(
      animation: _controller,
      builder: (BuildContext context, Widget? child) {
        return _buildMaterialIndicator(
          context,
          _strokeHeadTween.evaluate(_controller),
          _strokeTailTween.evaluate(_controller),
          _offsetTween.evaluate(_controller),
          _rotationTween.evaluate(_controller),
        );
      },
    );
  }

  @override
  Widget build(BuildContext context) {
    switch (widget._indicatorType) {
      case _ActivityIndicatorType.material:
        if (widget.value != null) {
          return _buildMaterialIndicator(context, 0.0, 0.0, 0, 0.0);
        }
        return _buildAnimation();
      case _ActivityIndicatorType.adaptive:
        final ThemeData theme = Theme.of(context);
        switch (theme.platform) {
          case TargetPlatform.iOS:
          case TargetPlatform.macOS:
            return _buildCupertinoIndicator(context);
          case TargetPlatform.android:
          case TargetPlatform.fuchsia:
          case TargetPlatform.linux:
          case TargetPlatform.windows:
            if (widget.value != null) {
              return _buildMaterialIndicator(context, 0.0, 0.0, 0, 0.0);
            }
            return _buildAnimation();
        }
    }
  }
}

class _RefreshProgressIndicatorPainter extends _CircularProgressIndicatorPainter {
  _RefreshProgressIndicatorPainter({
    required super.valueColor,
    required super.value,
    required super.headValue,
    required super.tailValue,
    required super.offsetValue,
    required super.rotationValue,
    required super.strokeWidth,
    required super.strokeAlign,
    required this.arrowheadScale,
    required super.strokeCap,
  });

  final double arrowheadScale;

  void paintArrowhead(Canvas canvas, Size size) {
    // ux, uy: a unit vector whose direction parallels the base of the arrowhead.
    // (So ux, -uy points in the direction the arrowhead points.)
    final double arcEnd = arcStart + arcSweep;
    final double ux = math.cos(arcEnd);
    final double uy = math.sin(arcEnd);

    assert(size.width == size.height);
    final double radius = size.width / 2.0;
    final double arrowheadPointX = radius + ux * radius + -uy * strokeWidth * 2.0 * arrowheadScale;
    final double arrowheadPointY = radius + uy * radius + ux * strokeWidth * 2.0 * arrowheadScale;
    final double arrowheadRadius = strokeWidth * 2.0 * arrowheadScale;
    final double innerRadius = radius - arrowheadRadius;
    final double outerRadius = radius + arrowheadRadius;

    final Path path =
        Path()
          ..moveTo(radius + ux * innerRadius, radius + uy * innerRadius)
          ..lineTo(radius + ux * outerRadius, radius + uy * outerRadius)
          ..lineTo(arrowheadPointX, arrowheadPointY)
          ..close();

    final Paint paint =
        Paint()
          ..color = valueColor
          ..strokeWidth = strokeWidth
          ..style = PaintingStyle.fill;
    canvas.drawPath(path, paint);
  }

  @override
  void paint(Canvas canvas, Size size) {
    super.paint(canvas, size);
    if (arrowheadScale > 0.0) {
      paintArrowhead(canvas, size);
    }
  }
}

/// An indicator for the progress of refreshing the contents of a widget.
///
/// Typically used for swipe-to-refresh interactions. See [RefreshIndicator] for
/// a complete implementation of swipe-to-refresh driven by a [Scrollable]
/// widget.
///
/// The indicator arc is displayed with [valueColor], an animated value. To
/// specify a constant color use: `AlwaysStoppedAnimation<Color>(color)`.
///
/// See also:
///
///  * [RefreshIndicator], which automatically displays a [CircularProgressIndicator]
///    when the underlying vertical scrollable is overscrolled.
class RefreshProgressIndicator extends CircularProgressIndicator {
  /// Creates a refresh progress indicator.
  ///
  /// Rather than creating a refresh progress indicator directly, consider using
  /// a [RefreshIndicator] together with a [Scrollable] widget.
  ///
  /// {@macro flutter.material.ProgressIndicator.ProgressIndicator}
  const RefreshProgressIndicator({
    super.key,
    super.value,
    super.backgroundColor,
    super.color,
    super.valueColor,
    super.strokeWidth = defaultStrokeWidth, // Different default than CircularProgressIndicator.
    super.strokeAlign,
    super.semanticsLabel,
    super.semanticsValue,
    super.strokeCap,
    this.elevation = 2.0,
    this.indicatorMargin = const EdgeInsets.all(4.0),
    this.indicatorPadding = const EdgeInsets.all(12.0),
  });

  /// {@macro flutter.material.material.elevation}
  final double elevation;

  /// The amount of space by which to inset the whole indicator.
  /// It accommodates the [elevation] of the indicator.
  final EdgeInsetsGeometry indicatorMargin;

  /// The amount of space by which to inset the inner refresh indicator.
  final EdgeInsetsGeometry indicatorPadding;

  /// Default stroke width.
  static const double defaultStrokeWidth = 2.5;

  /// {@template flutter.material.RefreshProgressIndicator.backgroundColor}
  /// Background color of that fills the circle under the refresh indicator.
  ///
  /// If [RefreshIndicator.backgroundColor] is null then the
  /// ambient [ProgressIndicatorThemeData.refreshBackgroundColor] will be used.
  /// If that is null, then the ambient theme's [ThemeData.canvasColor]
  /// will be used.
  /// {@endtemplate}
  @override
  Color? get backgroundColor => super.backgroundColor;

  @override
  State<CircularProgressIndicator> createState() => _RefreshProgressIndicatorState();
}

class _RefreshProgressIndicatorState extends _CircularProgressIndicatorState {
  static const double _indicatorSize = 41.0;

  /// Interval for arrow head to fully grow.
  static const double _strokeHeadInterval = 0.33;

  late final Animatable<double> _convertTween = CurveTween(
    curve: const Interval(0.1, _strokeHeadInterval),
  );

  late final Animatable<double> _additionalRotationTween = TweenSequence<double>(
    <TweenSequenceItem<double>>[
      // Makes arrow to expand a little bit earlier, to match the Android look.
      TweenSequenceItem<double>(
        tween: Tween<double>(begin: -0.1, end: -0.2),
        weight: _strokeHeadInterval,
      ),
      // Additional rotation after the arrow expanded
      TweenSequenceItem<double>(
        tween: Tween<double>(begin: -0.2, end: 1.35),
        weight: 1 - _strokeHeadInterval,
      ),
    ],
  );

  // Last value received from the widget before null.
  double? _lastValue;

  /// Force casting the widget as [RefreshProgressIndicator].
  @override
  RefreshProgressIndicator get widget => super.widget as RefreshProgressIndicator;

  // Always show the indeterminate version of the circular progress indicator.
  //
  // When value is non-null the sweep of the progress indicator arrow's arc
  // varies from 0 to about 300 degrees.
  //
  // When value is null the arrow animation starting from wherever we left it.
  @override
  Widget build(BuildContext context) {
    final double? value = widget.value;
    if (value != null) {
      _lastValue = value;
      _controller.value =
          _convertTween.transform(value) * (1333 / 2 / _kIndeterminateCircularDuration);
    }
    return _buildAnimation();
  }

  @override
  Widget _buildAnimation() {
    return AnimatedBuilder(
      animation: _controller,
      builder: (BuildContext context, Widget? child) {
        return _buildMaterialIndicator(
          context,
          // Lengthen the arc a little
          1.05 * _CircularProgressIndicatorState._strokeHeadTween.evaluate(_controller),
          _CircularProgressIndicatorState._strokeTailTween.evaluate(_controller),
          _CircularProgressIndicatorState._offsetTween.evaluate(_controller),
          _CircularProgressIndicatorState._rotationTween.evaluate(_controller),
        );
      },
    );
  }

  @override
  Widget _buildMaterialIndicator(
    BuildContext context,
    double headValue,
    double tailValue,
    double offsetValue,
    double rotationValue,
  ) {
    final double? value = widget.value;
    final double arrowheadScale =
        value == null ? 0.0 : const Interval(0.1, _strokeHeadInterval).transform(value);
    final double rotation;

    if (value == null && _lastValue == null) {
      rotation = 0.0;
    } else {
      rotation = math.pi * _additionalRotationTween.transform(value ?? _lastValue!);
    }

    Color valueColor = widget._getValueColor(context);
    final double opacity = valueColor.opacity;
    valueColor = valueColor.withOpacity(1.0);

    final ProgressIndicatorThemeData defaults = switch (Theme.of(context).useMaterial3) {
      true => _CircularProgressIndicatorDefaultsM3Year2023(context, indeterminate: value == null),
      false => _CircularProgressIndicatorDefaultsM2(context, indeterminate: value == null),
    };
    final ProgressIndicatorThemeData indicatorTheme = ProgressIndicatorTheme.of(context);
    final Color backgroundColor =
        widget.backgroundColor ??
        indicatorTheme.refreshBackgroundColor ??
        Theme.of(context).canvasColor;
    final double strokeWidth =
        widget.strokeWidth ?? indicatorTheme.strokeWidth ?? defaults.strokeWidth!;
    final double strokeAlign =
        widget.strokeAlign ?? indicatorTheme.strokeAlign ?? defaults.strokeAlign!;
    final StrokeCap? strokeCap = widget.strokeCap ?? indicatorTheme.strokeCap;

    return widget._buildSemanticsWrapper(
      context: context,
      child: Padding(
        padding: widget.indicatorMargin,
        child: SizedBox.fromSize(
          size: const Size.square(_indicatorSize),
          child: Material(
            type: MaterialType.circle,
            color: backgroundColor,
            elevation: widget.elevation,
            child: Padding(
              padding: widget.indicatorPadding,
              child: Opacity(
                opacity: opacity,
                child: Transform.rotate(
                  angle: rotation,
                  child: CustomPaint(
                    painter: _RefreshProgressIndicatorPainter(
                      valueColor: valueColor,
                      value: null, // Draw the indeterminate progress indicator.
                      headValue: headValue,
                      tailValue: tailValue,
                      offsetValue: offsetValue,
                      rotationValue: rotationValue,
                      strokeWidth: strokeWidth,
                      strokeAlign: strokeAlign,
                      arrowheadScale: arrowheadScale,
                      strokeCap: strokeCap,
                    ),
                  ),
                ),
              ),
            ),
          ),
        ),
      ),
    );
  }
}

// Hand coded defaults based on Material Design 2.
class _CircularProgressIndicatorDefaultsM2 extends ProgressIndicatorThemeData {
  _CircularProgressIndicatorDefaultsM2(this.context, {required this.indeterminate});

  final BuildContext context;
  late final ColorScheme _colors = Theme.of(context).colorScheme;
  final bool indeterminate;

  @override
  Color get color => _colors.primary;

  @override
  double? get strokeWidth => 4.0;

  @override
  double? get strokeAlign => CircularProgressIndicator.strokeAlignCenter;

  @override
  BoxConstraints get constraints => const BoxConstraints(minWidth: 36.0, minHeight: 36.0);
}

class _LinearProgressIndicatorDefaultsM2 extends ProgressIndicatorThemeData {
  _LinearProgressIndicatorDefaultsM2(this.context);

  final BuildContext context;
  late final ColorScheme _colors = Theme.of(context).colorScheme;

  @override
  Color get color => _colors.primary;

  @override
  Color get linearTrackColor => _colors.background;

  @override
  double get linearMinHeight => 4.0;
}

class _CircularProgressIndicatorDefaultsM3Year2023 extends ProgressIndicatorThemeData {
  _CircularProgressIndicatorDefaultsM3Year2023(this.context, {required this.indeterminate});

  final BuildContext context;
  late final ColorScheme _colors = Theme.of(context).colorScheme;
  final bool indeterminate;

  @override
  Color get color => _colors.primary;

  @override
  double get strokeWidth => 4.0;

  @override
  double? get strokeAlign => CircularProgressIndicator.strokeAlignCenter;

  @override
  BoxConstraints get constraints => const BoxConstraints(minWidth: 36.0, minHeight: 36.0);
}

class _LinearProgressIndicatorDefaultsM3Year2023 extends ProgressIndicatorThemeData {
  _LinearProgressIndicatorDefaultsM3Year2023(this.context);

  final BuildContext context;
  late final ColorScheme _colors = Theme.of(context).colorScheme;

  @override
  Color get color => _colors.primary;

  @override
  Color get linearTrackColor => _colors.secondaryContainer;

  @override
  double get linearMinHeight => 4.0;
}

// BEGIN GENERATED TOKEN PROPERTIES - ProgressIndicator

// Do not edit by hand. The code between the "BEGIN GENERATED" and
// "END GENERATED" comments are generated from data in the Material
// Design token database by the script:
//   dev/tools/gen_defaults/bin/gen_defaults.dart.

// dart format off
class _CircularProgressIndicatorDefaultsM3 extends ProgressIndicatorThemeData {
  _CircularProgressIndicatorDefaultsM3(this.context, { required this.indeterminate });

  final BuildContext context;
  late final ColorScheme _colors = Theme.of(context).colorScheme;
  final bool indeterminate;

  @override
  Color get color => _colors.primary;

  @override
  Color? get circularTrackColor => indeterminate ? null : _colors.secondaryContainer;

  @override
  double get strokeWidth => 4.0;

  @override
  double? get strokeAlign => CircularProgressIndicator.strokeAlignInside;

  @override
  BoxConstraints get constraints => const BoxConstraints(
    minWidth: 40.0,
    minHeight: 40.0,
  );

  @override
  double? get trackGap => 4.0;

  @override
  EdgeInsetsGeometry? get circularTrackPadding => const EdgeInsets.all(4.0);
}

class _LinearProgressIndicatorDefaultsM3 extends ProgressIndicatorThemeData {
  _LinearProgressIndicatorDefaultsM3(this.context);

  final BuildContext context;
  late final ColorScheme _colors = Theme.of(context).colorScheme;

  @override
  Color get color => _colors.primary;

  @override
  Color get linearTrackColor => _colors.secondaryContainer;

  @override
  double get linearMinHeight => 4.0;

  @override
  BorderRadius get borderRadius => BorderRadius.circular(4.0 / 2);

  @override
  Color get stopIndicatorColor => _colors.primary;

  @override
  double? get stopIndicatorRadius => 4.0 / 2;

  @override
  double? get trackGap => 4.0;
}
// dart format on

// END GENERATED TOKEN PROPERTIES - ProgressIndicator
