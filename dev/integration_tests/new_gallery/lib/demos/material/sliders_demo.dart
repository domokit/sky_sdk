// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math' as math;

import 'package:flutter/material.dart';
import '../../gallery_localizations.dart';
import 'material_demo_types.dart';

class SlidersDemo extends StatelessWidget {
  const SlidersDemo({super.key, required this.type});

  final SlidersDemoType type;

  String _title(BuildContext context) {
    final GalleryLocalizations localizations = GalleryLocalizations.of(context)!;
    switch (type) {
      case SlidersDemoType.sliders:
        return localizations.demoSlidersTitle;
      case SlidersDemoType.rangeSliders:
        return localizations.demoRangeSlidersTitle;
      case SlidersDemoType.customSliders:
        return localizations.demoCustomSlidersTitle;
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(automaticallyImplyLeading: false, title: Text(_title(context))),
      body: switch (type) {
        SlidersDemoType.sliders => _Sliders(),
        SlidersDemoType.rangeSliders => _RangeSliders(),
        SlidersDemoType.customSliders => _CustomSliders(),
      },
    );
  }
}
// BEGIN slidersDemo

class _Sliders extends StatefulWidget {
  @override
  _SlidersState createState() => _SlidersState();
}

class _SlidersState extends State<_Sliders> with RestorationMixin {
  final RestorableDouble _continuousValue = RestorableDouble(25);
  final RestorableDouble _discreteValue = RestorableDouble(20);

  @override
  String get restorationId => 'slider_demo';

  @override
  void restoreState(RestorationBucket? oldBucket, bool initialRestore) {
    registerForRestoration(_continuousValue, 'continuous_value');
    registerForRestoration(_discreteValue, 'discrete_value');
  }

  @override
  void dispose() {
    _continuousValue.dispose();
    _discreteValue.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final GalleryLocalizations localizations = GalleryLocalizations.of(context)!;
    return Padding(
      padding: const EdgeInsets.symmetric(horizontal: 40),
      child: Column(
        mainAxisAlignment: MainAxisAlignment.center,
        children: <Widget>[
          Column(
            mainAxisSize: MainAxisSize.min,
            children: <Widget>[
              Semantics(
                label: localizations.demoSlidersEditableNumericalValue,
                child: SizedBox(
                  width: 64,
                  height: 48,
                  child: TextField(
                    textAlign: TextAlign.center,
                    onSubmitted: (String value) {
                      final double? newValue = double.tryParse(value);
                      if (newValue != null && newValue != _continuousValue.value) {
                        setState(() {
                          _continuousValue.value = newValue.clamp(0, 100) as double;
                        });
                      }
                    },
                    keyboardType: TextInputType.number,
                    controller: TextEditingController(
                      text: _continuousValue.value.toStringAsFixed(0),
                    ),
                  ),
                ),
              ),
              Slider(
                value: _continuousValue.value,
                max: 100,
                onChanged: (double value) {
                  setState(() {
                    _continuousValue.value = value;
                  });
                },
              ),
              // Disabled slider
              Slider(value: _continuousValue.value, max: 100, onChanged: null),
              Text(localizations.demoSlidersContinuousWithEditableNumericalValue),
            ],
          ),
          const SizedBox(height: 80),
          Column(
            mainAxisSize: MainAxisSize.min,
            children: <Widget>[
              Slider(
                value: _discreteValue.value,
                max: 200,
                divisions: 5,
                label: _discreteValue.value.round().toString(),
                onChanged: (double value) {
                  setState(() {
                    _discreteValue.value = value;
                  });
                },
              ),
              // Disabled slider
              Slider(
                value: _discreteValue.value,
                max: 200,
                divisions: 5,
                label: _discreteValue.value.round().toString(),
                onChanged: null,
              ),
              Text(localizations.demoSlidersDiscrete),
            ],
          ),
        ],
      ),
    );
  }
}

// END

// BEGIN rangeSlidersDemo

class _RangeSliders extends StatefulWidget {
  @override
  _RangeSlidersState createState() => _RangeSlidersState();
}

class _RangeSlidersState extends State<_RangeSliders> with RestorationMixin {
  final RestorableDouble _continuousStartValue = RestorableDouble(25);
  final RestorableDouble _continuousEndValue = RestorableDouble(75);
  final RestorableDouble _discreteStartValue = RestorableDouble(40);
  final RestorableDouble _discreteEndValue = RestorableDouble(120);

  @override
  String get restorationId => 'range_sliders_demo';

  @override
  void restoreState(RestorationBucket? oldBucket, bool initialRestore) {
    registerForRestoration(_continuousStartValue, 'continuous_start_value');
    registerForRestoration(_continuousEndValue, 'continuous_end_value');
    registerForRestoration(_discreteStartValue, 'discrete_start_value');
    registerForRestoration(_discreteEndValue, 'discrete_end_value');
  }

  @override
  void dispose() {
    _continuousStartValue.dispose();
    _continuousEndValue.dispose();
    _discreteStartValue.dispose();
    _discreteEndValue.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final RangeValues continuousValues = RangeValues(
      _continuousStartValue.value,
      _continuousEndValue.value,
    );
    final RangeValues discreteValues = RangeValues(
      _discreteStartValue.value,
      _discreteEndValue.value,
    );

    return Padding(
      padding: const EdgeInsets.symmetric(horizontal: 40),
      child: Column(
        mainAxisAlignment: MainAxisAlignment.center,
        children: <Widget>[
          Column(
            mainAxisSize: MainAxisSize.min,
            children: <Widget>[
              RangeSlider(
                values: continuousValues,
                max: 100,
                onChanged: (RangeValues values) {
                  setState(() {
                    _continuousStartValue.value = values.start;
                    _continuousEndValue.value = values.end;
                  });
                },
              ),
              // Disabled range slider
              RangeSlider(values: continuousValues, max: 100, onChanged: null),
              Text(GalleryLocalizations.of(context)!.demoSlidersContinuous),
            ],
          ),
          const SizedBox(height: 80),
          Column(
            mainAxisSize: MainAxisSize.min,
            children: <Widget>[
              RangeSlider(
                values: discreteValues,
                max: 200,
                divisions: 5,
                labels: RangeLabels(
                  discreteValues.start.round().toString(),
                  discreteValues.end.round().toString(),
                ),
                onChanged: (RangeValues values) {
                  setState(() {
                    _discreteStartValue.value = values.start;
                    _discreteEndValue.value = values.end;
                  });
                },
              ),
              // Disabled range slider
              RangeSlider(
                values: discreteValues,
                max: 200,
                divisions: 5,
                labels: RangeLabels(
                  discreteValues.start.round().toString(),
                  discreteValues.end.round().toString(),
                ),
                onChanged: null,
              ),
              Text(GalleryLocalizations.of(context)!.demoSlidersDiscrete),
            ],
          ),
        ],
      ),
    );
  }
}

// END

// BEGIN customSlidersDemo

Path _downTriangle(double size, Offset thumbCenter, {bool invert = false}) {
  final Path thumbPath = Path();
  final double height = math.sqrt(3) / 2;
  final double centerHeight = size * height / 3;
  final double halfSize = size / 2;
  final int sign = invert ? -1 : 1;
  thumbPath.moveTo(thumbCenter.dx - halfSize, thumbCenter.dy + sign * centerHeight);
  thumbPath.lineTo(thumbCenter.dx, thumbCenter.dy - 2 * sign * centerHeight);
  thumbPath.lineTo(thumbCenter.dx + halfSize, thumbCenter.dy + sign * centerHeight);
  thumbPath.close();
  return thumbPath;
}

Path _rightTriangle(double size, Offset thumbCenter, {bool invert = false}) {
  final Path thumbPath = Path();
  final double halfSize = size / 2;
  final int sign = invert ? -1 : 1;
  thumbPath.moveTo(thumbCenter.dx + halfSize * sign, thumbCenter.dy);
  thumbPath.lineTo(thumbCenter.dx - halfSize * sign, thumbCenter.dy - size);
  thumbPath.lineTo(thumbCenter.dx - halfSize * sign, thumbCenter.dy + size);
  thumbPath.close();
  return thumbPath;
}

Path _upTriangle(double size, Offset thumbCenter) => _downTriangle(size, thumbCenter, invert: true);

Path _leftTriangle(double size, Offset thumbCenter) =>
    _rightTriangle(size, thumbCenter, invert: true);

class _CustomRangeThumbShape extends RangeSliderThumbShape {
  const _CustomRangeThumbShape();

  static const double _thumbSize = 4;
  static const double _disabledThumbSize = 3;

  @override
  Size getPreferredSize(bool isEnabled, bool isDiscrete) {
    return isEnabled
        ? const Size.fromRadius(_thumbSize)
        : const Size.fromRadius(_disabledThumbSize);
  }

  static final Animatable<double> sizeTween = Tween<double>(
    begin: _disabledThumbSize,
    end: _thumbSize,
  );

  @override
  void paint(
    PaintingContext context,
    Offset center, {
    required Animation<double> activationAnimation,
    required Animation<double> enableAnimation,
    bool isDiscrete = false,
    bool isEnabled = false,
    bool? isOnTop,
    TextDirection? textDirection,
    required SliderThemeData sliderTheme,
    Thumb? thumb,
    bool? isPressed,
  }) {
    final Canvas canvas = context.canvas;
    final ColorTween colorTween = ColorTween(
      begin: sliderTheme.disabledThumbColor,
      end: sliderTheme.thumbColor,
    );

    final double size = _thumbSize * sizeTween.evaluate(enableAnimation);
    canvas.drawPath(switch ((textDirection!, thumb!)) {
      (TextDirection.rtl, Thumb.start) => _rightTriangle(size, center),
      (TextDirection.rtl, Thumb.end) => _leftTriangle(size, center),
      (TextDirection.ltr, Thumb.start) => _leftTriangle(size, center),
      (TextDirection.ltr, Thumb.end) => _rightTriangle(size, center),
    }, Paint()..color = colorTween.evaluate(enableAnimation)!);
  }
}

class _CustomThumbShape extends SliderComponentShape {
  const _CustomThumbShape();

  static const double _thumbSize = 4;
  static const double _disabledThumbSize = 3;

  @override
  Size getPreferredSize(bool isEnabled, bool isDiscrete) {
    return isEnabled
        ? const Size.fromRadius(_thumbSize)
        : const Size.fromRadius(_disabledThumbSize);
  }

  static final Animatable<double> sizeTween = Tween<double>(
    begin: _disabledThumbSize,
    end: _thumbSize,
  );

  @override
  void paint(
    PaintingContext context,
    Offset thumbCenter, {
    Animation<double>? activationAnimation,
    required Animation<double> enableAnimation,
    bool? isDiscrete,
    TextPainter? labelPainter,
    RenderBox? parentBox,
    required SliderThemeData sliderTheme,
    TextDirection? textDirection,
    double? value,
    double? textScaleFactor,
    Size? sizeWithOverflow,
  }) {
    final Canvas canvas = context.canvas;
    final ColorTween colorTween = ColorTween(
      begin: sliderTheme.disabledThumbColor,
      end: sliderTheme.thumbColor,
    );
    final double size = _thumbSize * sizeTween.evaluate(enableAnimation);
    final Path thumbPath = _downTriangle(size, thumbCenter);
    canvas.drawPath(thumbPath, Paint()..color = colorTween.evaluate(enableAnimation)!);
  }
}

class _CustomValueIndicatorShape extends SliderComponentShape {
  const _CustomValueIndicatorShape();

  static const double _indicatorSize = 4;
  static const double _disabledIndicatorSize = 3;
  static const double _slideUpHeight = 40;

  @override
  Size getPreferredSize(bool isEnabled, bool isDiscrete) {
    return Size.fromRadius(isEnabled ? _indicatorSize : _disabledIndicatorSize);
  }

  static final Animatable<double> sizeTween = Tween<double>(
    begin: _disabledIndicatorSize,
    end: _indicatorSize,
  );

  @override
  void paint(
    PaintingContext context,
    Offset thumbCenter, {
    required Animation<double> activationAnimation,
    required Animation<double> enableAnimation,
    bool? isDiscrete,
    required TextPainter labelPainter,
    RenderBox? parentBox,
    required SliderThemeData sliderTheme,
    TextDirection? textDirection,
    double? value,
    double? textScaleFactor,
    Size? sizeWithOverflow,
  }) {
    final Canvas canvas = context.canvas;
    final ColorTween enableColor = ColorTween(
      begin: sliderTheme.disabledThumbColor,
      end: sliderTheme.valueIndicatorColor,
    );
    final Tween<double> slideUpTween = Tween<double>(begin: 0, end: _slideUpHeight);
    final double size = _indicatorSize * sizeTween.evaluate(enableAnimation);
    final Offset slideUpOffset = Offset(0, -slideUpTween.evaluate(activationAnimation));
    final Path thumbPath = _upTriangle(size, thumbCenter + slideUpOffset);
    final Color paintColor = enableColor
        .evaluate(enableAnimation)!
        .withAlpha((255 * activationAnimation.value).round());
    canvas.drawPath(thumbPath, Paint()..color = paintColor);
    canvas.drawLine(
      thumbCenter,
      thumbCenter + slideUpOffset,
      Paint()
        ..color = paintColor
        ..style = PaintingStyle.stroke
        ..strokeWidth = 2,
    );
    labelPainter.paint(
      canvas,
      thumbCenter + slideUpOffset + Offset(-labelPainter.width / 2, -labelPainter.height - 4),
    );
  }
}

class _CustomSliders extends StatefulWidget {
  @override
  _CustomSlidersState createState() => _CustomSlidersState();
}

class _CustomSlidersState extends State<_CustomSliders> with RestorationMixin {
  final RestorableDouble _continuousStartCustomValue = RestorableDouble(40);
  final RestorableDouble _continuousEndCustomValue = RestorableDouble(160);
  final RestorableDouble _discreteCustomValue = RestorableDouble(25);

  @override
  String get restorationId => 'custom_sliders_demo';

  @override
  void restoreState(RestorationBucket? oldBucket, bool initialRestore) {
    registerForRestoration(_continuousStartCustomValue, 'continuous_start_custom_value');
    registerForRestoration(_continuousEndCustomValue, 'continuous_end_custom_value');
    registerForRestoration(_discreteCustomValue, 'discrete_custom_value');
  }

  @override
  void dispose() {
    _continuousStartCustomValue.dispose();
    _continuousEndCustomValue.dispose();
    _discreteCustomValue.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final RangeValues customRangeValue = RangeValues(
      _continuousStartCustomValue.value,
      _continuousEndCustomValue.value,
    );
    final ThemeData theme = Theme.of(context);
    final GalleryLocalizations localizations = GalleryLocalizations.of(context)!;
    return Padding(
      padding: const EdgeInsets.symmetric(horizontal: 40),
      child: Column(
        mainAxisAlignment: MainAxisAlignment.center,
        children: <Widget>[
          Column(
            mainAxisSize: MainAxisSize.min,
            children: <Widget>[
              SliderTheme(
                data: theme.sliderTheme.copyWith(
                  trackHeight: 2,
                  activeTrackColor: Colors.deepPurple,
                  inactiveTrackColor: theme.colorScheme.onSurface.withOpacity(0.5),
                  activeTickMarkColor: theme.colorScheme.onSurface.withOpacity(0.7),
                  inactiveTickMarkColor: theme.colorScheme.surface.withOpacity(0.7),
                  overlayColor: theme.colorScheme.onSurface.withOpacity(0.12),
                  thumbColor: Colors.deepPurple,
                  valueIndicatorColor: Colors.deepPurpleAccent,
                  thumbShape: const _CustomThumbShape(),
                  valueIndicatorShape: const _CustomValueIndicatorShape(),
                  valueIndicatorTextStyle: theme.textTheme.bodyLarge!.copyWith(
                    color: theme.colorScheme.onSurface,
                  ),
                ),
                child: Slider(
                  value: _discreteCustomValue.value,
                  max: 200,
                  divisions: 5,
                  semanticFormatterCallback: (double value) => value.round().toString(),
                  label: '${_discreteCustomValue.value.round()}',
                  onChanged: (double value) {
                    setState(() {
                      _discreteCustomValue.value = value;
                    });
                  },
                ),
              ),
              Text(localizations.demoSlidersDiscreteSliderWithCustomTheme),
            ],
          ),
          const SizedBox(height: 80),
          Column(
            mainAxisSize: MainAxisSize.min,
            children: <Widget>[
              SliderTheme(
                data: const SliderThemeData(
                  trackHeight: 2,
                  activeTrackColor: Colors.deepPurple,
                  inactiveTrackColor: Colors.black26,
                  activeTickMarkColor: Colors.white70,
                  inactiveTickMarkColor: Colors.black,
                  overlayColor: Colors.black12,
                  thumbColor: Colors.deepPurple,
                  rangeThumbShape: _CustomRangeThumbShape(),
                  showValueIndicator: ShowValueIndicator.never,
                ),
                child: RangeSlider(
                  values: customRangeValue,
                  max: 200,
                  onChanged: (RangeValues values) {
                    setState(() {
                      _continuousStartCustomValue.value = values.start;
                      _continuousEndCustomValue.value = values.end;
                    });
                  },
                ),
              ),
              Text(localizations.demoSlidersContinuousRangeSliderWithCustomTheme),
            ],
          ),
        ],
      ),
    );
  }
}

// END
