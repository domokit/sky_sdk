// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math' as math;

import 'package:flutter/material.dart';

import '../../gallery/demo.dart';

class SliderDemo extends StatefulWidget {
  const SliderDemo({super.key});

  static const String routeName = '/material/slider';

  @override
  State<SliderDemo> createState() => _SliderDemoState();
}

Path _downTriangle(double size, Offset thumbCenter, {bool invert = false}) {
  final Path thumbPath = Path();
  final double height = math.sqrt(3.0) / 2.0;
  final double centerHeight = size * height / 3.0;
  final double halfSize = size / 2.0;
  final double sign = invert ? -1.0 : 1.0;
  thumbPath.moveTo(thumbCenter.dx - halfSize, thumbCenter.dy + sign * centerHeight);
  thumbPath.lineTo(thumbCenter.dx, thumbCenter.dy - 2.0 * sign * centerHeight);
  thumbPath.lineTo(thumbCenter.dx + halfSize, thumbCenter.dy + sign * centerHeight);
  thumbPath.close();
  return thumbPath;
}

Path _rightTriangle(double size, Offset thumbCenter, {bool invert = false}) {
  final Path thumbPath = Path();
  final double halfSize = size / 2.0;
  final double sign = invert ? -1.0 : 1.0;
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
  static const double _thumbSize = 4.0;
  static const double _disabledThumbSize = 3.0;

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
    required SliderThemeData sliderTheme,
    TextDirection? textDirection,
    Thumb? thumb,
    bool? isPressed,
  }) {
    final Canvas canvas = context.canvas;
    final ColorTween colorTween = ColorTween(
      begin: sliderTheme.disabledThumbColor,
      end: sliderTheme.thumbColor,
    );

    final double size = _thumbSize * sizeTween.evaluate(enableAnimation);
    final Path thumbPath = switch ((textDirection!, thumb!)) {
      (TextDirection.rtl, Thumb.start) => _rightTriangle(size, center),
      (TextDirection.rtl, Thumb.end) => _leftTriangle(size, center),
      (TextDirection.ltr, Thumb.start) => _leftTriangle(size, center),
      (TextDirection.ltr, Thumb.end) => _rightTriangle(size, center),
    };
    canvas.drawPath(thumbPath, Paint()..color = colorTween.evaluate(enableAnimation)!);
  }
}

class _CustomThumbShape extends SliderComponentShape {
  static const double _thumbSize = 4.0;
  static const double _disabledThumbSize = 3.0;

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
  static const double _indicatorSize = 4.0;
  static const double _disabledIndicatorSize = 3.0;
  static const double _slideUpHeight = 40.0;

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
    final Tween<double> slideUpTween = Tween<double>(begin: 0.0, end: _slideUpHeight);
    final double size = _indicatorSize * sizeTween.evaluate(enableAnimation);
    final Offset slideUpOffset = Offset(0.0, -slideUpTween.evaluate(activationAnimation));
    final Path thumbPath = _upTriangle(size, thumbCenter + slideUpOffset);
    final Color paintColor = enableColor
        .evaluate(enableAnimation)!
        .withAlpha((255.0 * activationAnimation.value).round());
    canvas.drawPath(thumbPath, Paint()..color = paintColor);
    canvas.drawLine(
      thumbCenter,
      thumbCenter + slideUpOffset,
      Paint()
        ..color = paintColor
        ..style = PaintingStyle.stroke
        ..strokeWidth = 2.0,
    );
    labelPainter.paint(
      canvas,
      thumbCenter + slideUpOffset + Offset(-labelPainter.width / 2.0, -labelPainter.height - 4.0),
    );
  }
}

class _SliderDemoState extends State<SliderDemo> {
  @override
  Widget build(BuildContext context) {
    const List<ComponentDemoTabData> demos = <ComponentDemoTabData>[
      ComponentDemoTabData(
        tabName: 'SINGLE',
        description: 'Sliders containing 1 thumb',
        demoWidget: _Sliders(),
        documentationUrl: 'https://api.flutter.dev/flutter/material/Slider-class.html',
      ),
      ComponentDemoTabData(
        tabName: 'RANGE',
        description: 'Sliders containing 2 thumbs',
        demoWidget: _RangeSliders(),
        documentationUrl: 'https://api.flutter.dev/flutter/material/RangeSlider-class.html',
      ),
    ];

    return const TabbedComponentDemoScaffold(
      title: 'Sliders',
      demos: demos,
      isScrollable: false,
      showExampleCodeAction: false,
    );
  }
}

class _Sliders extends StatefulWidget {
  const _Sliders();

  @override
  _SlidersState createState() => _SlidersState();
}

class _SlidersState extends State<_Sliders> {
  double _continuousValue = 25.0;
  double _discreteValue = 20.0;
  double _discreteCustomValue = 25.0;

  @override
  Widget build(BuildContext context) {
    final ThemeData theme = Theme.of(context);
    return Padding(
      padding: const EdgeInsets.symmetric(horizontal: 40.0),
      child: Column(
        mainAxisAlignment: MainAxisAlignment.spaceAround,
        children: <Widget>[
          Column(
            mainAxisSize: MainAxisSize.min,
            children: <Widget>[
              Semantics(
                label: 'Editable numerical value',
                child: SizedBox(
                  width: 64,
                  height: 48,
                  child: TextField(
                    textAlign: TextAlign.center,
                    onSubmitted: (String value) {
                      final double? newValue = double.tryParse(value);
                      if (newValue != null && newValue != _continuousValue) {
                        setState(() {
                          _continuousValue = newValue.clamp(0.0, 100.0);
                        });
                      }
                    },
                    keyboardType: TextInputType.number,
                    controller: TextEditingController(text: _continuousValue.toStringAsFixed(0)),
                  ),
                ),
              ),
              Slider.adaptive(
                label: _continuousValue.toStringAsFixed(6),
                value: _continuousValue,
                max: 100.0,
                onChanged: (double value) {
                  setState(() {
                    _continuousValue = value;
                  });
                },
              ),
              const Text('Continuous with Editable Numerical Value'),
            ],
          ),
          const Column(
            mainAxisSize: MainAxisSize.min,
            children: <Widget>[Slider.adaptive(value: 0.25, onChanged: null), Text('Disabled')],
          ),
          Column(
            mainAxisSize: MainAxisSize.min,
            children: <Widget>[
              Slider.adaptive(
                value: _discreteValue,
                max: 200.0,
                divisions: 5,
                label: '${_discreteValue.round()}',
                onChanged: (double value) {
                  setState(() {
                    _discreteValue = value;
                  });
                },
              ),
              const Text('Discrete'),
            ],
          ),
          Column(
            mainAxisSize: MainAxisSize.min,
            children: <Widget>[
              SliderTheme(
                data: theme.sliderTheme.copyWith(
                  activeTrackColor: Colors.deepPurple,
                  inactiveTrackColor: theme.colorScheme.onSurface.withOpacity(0.5),
                  activeTickMarkColor: theme.colorScheme.onSurface.withOpacity(0.7),
                  inactiveTickMarkColor: theme.colorScheme.surface.withOpacity(0.7),
                  overlayColor: theme.colorScheme.onSurface.withOpacity(0.12),
                  thumbColor: Colors.deepPurple,
                  valueIndicatorColor: Colors.deepPurpleAccent,
                  thumbShape: _CustomThumbShape(),
                  valueIndicatorShape: _CustomValueIndicatorShape(),
                  valueIndicatorTextStyle: theme.textTheme.bodyLarge!.copyWith(
                    color: theme.colorScheme.onSurface,
                  ),
                ),
                child: Slider(
                  value: _discreteCustomValue,
                  max: 200.0,
                  divisions: 5,
                  semanticFormatterCallback: (double value) => value.round().toString(),
                  label: '${_discreteCustomValue.round()}',
                  onChanged: (double value) {
                    setState(() {
                      _discreteCustomValue = value;
                    });
                  },
                ),
              ),
              const Text('Discrete with Custom Theme'),
            ],
          ),
        ],
      ),
    );
  }
}

class _RangeSliders extends StatefulWidget {
  const _RangeSliders();

  @override
  _RangeSlidersState createState() => _RangeSlidersState();
}

class _RangeSlidersState extends State<_RangeSliders> {
  RangeValues _continuousValues = const RangeValues(25.0, 75.0);
  RangeValues _discreteValues = const RangeValues(40.0, 120.0);
  RangeValues _discreteCustomValues = const RangeValues(40.0, 160.0);

  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.symmetric(horizontal: 40.0),
      child: Column(
        mainAxisAlignment: MainAxisAlignment.spaceAround,
        children: <Widget>[
          Column(
            mainAxisSize: MainAxisSize.min,
            children: <Widget>[
              RangeSlider(
                values: _continuousValues,
                max: 100.0,
                onChanged: (RangeValues values) {
                  setState(() {
                    _continuousValues = values;
                  });
                },
              ),
              const Text('Continuous'),
            ],
          ),
          Column(
            mainAxisSize: MainAxisSize.min,
            children: <Widget>[
              RangeSlider(values: const RangeValues(0.25, 0.75), onChanged: null),
              const Text('Disabled'),
            ],
          ),
          Column(
            mainAxisSize: MainAxisSize.min,
            children: <Widget>[
              RangeSlider(
                values: _discreteValues,
                max: 200.0,
                divisions: 5,
                labels: RangeLabels(
                  '${_discreteValues.start.round()}',
                  '${_discreteValues.end.round()}',
                ),
                onChanged: (RangeValues values) {
                  setState(() {
                    _discreteValues = values;
                  });
                },
              ),
              const Text('Discrete'),
            ],
          ),
          Column(
            mainAxisSize: MainAxisSize.min,
            children: <Widget>[
              SliderTheme(
                data: SliderThemeData(
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
                  values: _discreteCustomValues,
                  max: 200.0,
                  divisions: 5,
                  labels: RangeLabels(
                    '${_discreteCustomValues.start.round()}',
                    '${_discreteCustomValues.end.round()}',
                  ),
                  onChanged: (RangeValues values) {
                    setState(() {
                      _discreteCustomValues = values;
                    });
                  },
                ),
              ),
              const Text('Discrete with Custom Theme'),
            ],
          ),
        ],
      ),
    );
  }
}
