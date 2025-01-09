// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// An example showing usage of [StarBorder].

import 'package:flutter/material.dart';

const int _kParameterPrecision = 2;

void main() => runApp(const StarBorderApp());

class StarBorderApp extends StatelessWidget {
  const StarBorderApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(
          title: const Text('StarBorder Example'),
          backgroundColor: const Color(0xff323232),
        ),
        body: const StarBorderExample(),
      ),
    );
  }
}

class StarBorderExample extends StatefulWidget {
  const StarBorderExample({super.key});

  @override
  State<StarBorderExample> createState() => _StarBorderExampleState();
}

class _StarBorderExampleState extends State<StarBorderExample> {
  final OptionModel _model = OptionModel();
  final TextEditingController _textController = TextEditingController();

  @override
  void initState() {
    super.initState();
    _model.addListener(_modelChanged);
  }

  @override
  void dispose() {
    _model.removeListener(_modelChanged);
    _textController.dispose();
    super.dispose();
  }

  void _modelChanged() {
    setState(() {});
  }

  @override
  Widget build(BuildContext context) {
    return DefaultTextStyle(
      style: const TextStyle(
        color: Colors.black,
        fontSize: 14.0,
        fontFamily: 'Roboto',
        fontStyle: FontStyle.normal,
      ),
      child: ListView(
        children: <Widget>[
          ColoredBox(color: Colors.grey.shade200, child: Options(_model)),
          Padding(
            padding: const EdgeInsets.all(18.0),
            child: Row(
              mainAxisAlignment: MainAxisAlignment.spaceEvenly,
              children: <Widget>[
                Expanded(
                  child: ExampleBorder(
                    border: StarBorder(
                      side: const BorderSide(),
                      points: _model.points,
                      innerRadiusRatio: _model.innerRadiusRatio,
                      pointRounding: _model.pointRounding,
                      valleyRounding: _model.valleyRounding,
                      rotation: _model.rotation,
                      squash: _model.squash,
                    ),
                    title: 'Star',
                  ),
                ),
                Expanded(
                  child: ExampleBorder(
                    border: StarBorder.polygon(
                      side: const BorderSide(),
                      sides: _model.points,
                      pointRounding: _model.pointRounding,
                      rotation: _model.rotation,
                      squash: _model.squash,
                    ),
                    title: 'Polygon',
                  ),
                ),
              ],
            ),
          ),
          Row(
            crossAxisAlignment: CrossAxisAlignment.start,
            children: <Widget>[
              Expanded(
                child: Container(
                  color: Colors.black12,
                  margin: const EdgeInsets.all(16.0),
                  padding: const EdgeInsets.all(16.0),
                  child: SelectableText(_model.starCode),
                ),
              ),
              Expanded(
                child: Container(
                  color: Colors.black12,
                  margin: const EdgeInsets.all(16.0),
                  padding: const EdgeInsets.all(16.0),
                  child: SelectableText(_model.polygonCode),
                ),
              ),
            ],
          ),
        ],
      ),
    );
  }
}

class ExampleBorder extends StatelessWidget {
  const ExampleBorder({super.key, required this.border, required this.title});

  final StarBorder border;
  final String title;

  @override
  Widget build(BuildContext context) {
    return Container(
      alignment: Alignment.center,
      padding: const EdgeInsets.all(20),
      width: 150,
      height: 100,
      decoration: ShapeDecoration(color: Colors.blue.shade100, shape: border),
      child: Text(title),
    );
  }
}

class Options extends StatefulWidget {
  const Options(this.model, {super.key});

  final OptionModel model;

  @override
  State<Options> createState() => _OptionsState();
}

class _OptionsState extends State<Options> {
  @override
  void initState() {
    super.initState();
    widget.model.addListener(_modelChanged);
  }

  @override
  void didUpdateWidget(Options oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (widget.model != oldWidget.model) {
      oldWidget.model.removeListener(_modelChanged);
      widget.model.addListener(_modelChanged);
    }
  }

  @override
  void dispose() {
    super.dispose();
    widget.model.removeListener(_modelChanged);
  }

  void _modelChanged() {
    setState(() {});
  }

  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.fromLTRB(5.0, 0.0, 5.0, 10.0),
      child: Column(
        mainAxisSize: MainAxisSize.min,
        children: <Widget>[
          Row(
            children: <Widget>[
              Expanded(
                child: ControlSlider(
                  label: 'Point Rounding',
                  value: widget.model.pointRounding,
                  onChanged: (double value) {
                    widget.model.pointRounding = value;
                  },
                ),
              ),
              Expanded(
                child: ControlSlider(
                  label: 'Valley Rounding',
                  value: widget.model.valleyRounding,
                  onChanged: (double value) {
                    widget.model.valleyRounding = value;
                  },
                ),
              ),
            ],
          ),
          Row(
            children: <Widget>[
              Expanded(
                child: ControlSlider(
                  label: 'Squash',
                  value: widget.model.squash,
                  onChanged: (double value) {
                    widget.model.squash = value;
                  },
                ),
              ),
              Expanded(
                child: ControlSlider(
                  label: 'Rotation',
                  value: widget.model.rotation,
                  max: 360,
                  onChanged: (double value) {
                    widget.model.rotation = value;
                  },
                ),
              ),
            ],
          ),
          Row(
            children: <Widget>[
              Expanded(
                child: Row(
                  children: <Widget>[
                    Expanded(
                      child: ControlSlider(
                        label: 'Points',
                        value: widget.model.points,
                        min: 3,
                        max: 20,
                        precision: 1,
                        onChanged: (double value) {
                          widget.model.points = value;
                        },
                      ),
                    ),
                    Tooltip(
                      message: 'Round the number of points to the nearest integer.',
                      child: Padding(
                        padding: const EdgeInsets.all(8.0),
                        child: OutlinedButton(
                          child: const Text('Nearest'),
                          onPressed: () {
                            widget.model.points = widget.model.points.roundToDouble();
                          },
                        ),
                      ),
                    ),
                  ],
                ),
              ),
              Expanded(
                child: ControlSlider(
                  label: 'Inner Radius',
                  value: widget.model.innerRadiusRatio,
                  onChanged: (double value) {
                    widget.model.innerRadiusRatio = value;
                  },
                ),
              ),
            ],
          ),
          ElevatedButton(
            onPressed: () {
              widget.model.reset();
            },
            child: const Text('Reset'),
          ),
        ],
      ),
    );
  }
}

class OptionModel extends ChangeNotifier {
  double get pointRounding => _pointRounding;
  double _pointRounding = 0.0;
  set pointRounding(double value) {
    if (value != _pointRounding) {
      _pointRounding = value;
      if (_valleyRounding + _pointRounding > 1) {
        _valleyRounding = 1.0 - _pointRounding;
      }
      notifyListeners();
    }
  }

  double get valleyRounding => _valleyRounding;
  double _valleyRounding = 0.0;
  set valleyRounding(double value) {
    if (value != _valleyRounding) {
      _valleyRounding = value;
      if (_valleyRounding + _pointRounding > 1) {
        _pointRounding = 1.0 - _valleyRounding;
      }
      notifyListeners();
    }
  }

  double get squash => _squash;
  double _squash = 0.0;
  set squash(double value) {
    if (value != _squash) {
      _squash = value;
      notifyListeners();
    }
  }

  double get rotation => _rotation;
  double _rotation = 0.0;
  set rotation(double value) {
    if (value != _rotation) {
      _rotation = value;
      notifyListeners();
    }
  }

  double get innerRadiusRatio => _innerRadiusRatio;
  double _innerRadiusRatio = 0.4;
  set innerRadiusRatio(double value) {
    if (value != _innerRadiusRatio) {
      _innerRadiusRatio = value.clamp(0.0001, double.infinity);
      notifyListeners();
    }
  }

  double get points => _points;
  double _points = 5;
  set points(double value) {
    if (value != _points) {
      _points = value;
      notifyListeners();
    }
  }

  String get starCode {
    return 'Container(\n'
        '  decoration: ShapeDecoration(\n'
        '    shape: StarBorder(\n'
        '      points: ${points.toStringAsFixed(_kParameterPrecision)},\n'
        '      rotation: ${rotation.toStringAsFixed(_kParameterPrecision)},\n'
        '      innerRadiusRatio: ${innerRadiusRatio.toStringAsFixed(_kParameterPrecision)},\n'
        '      pointRounding: ${pointRounding.toStringAsFixed(_kParameterPrecision)},\n'
        '      valleyRounding: ${valleyRounding.toStringAsFixed(_kParameterPrecision)},\n'
        '      squash: ${squash.toStringAsFixed(_kParameterPrecision)},\n'
        '    ),\n'
        '  ),\n'
        ');';
  }

  String get polygonCode {
    return 'Container(\n'
        '  decoration: ShapeDecoration(\n'
        '    shape: StarBorder.polygon(\n'
        '      sides: ${points.toStringAsFixed(_kParameterPrecision)},\n'
        '      rotation: ${rotation.toStringAsFixed(_kParameterPrecision)},\n'
        '      cornerRounding: ${pointRounding.toStringAsFixed(_kParameterPrecision)},\n'
        '      squash: ${squash.toStringAsFixed(_kParameterPrecision)},\n'
        '    ),\n'
        '  ),\n'
        ');';
  }

  void reset() {
    final OptionModel defaultModel = OptionModel();
    _pointRounding = defaultModel.pointRounding;
    _valleyRounding = defaultModel.valleyRounding;
    _rotation = defaultModel.rotation;
    _squash = defaultModel.squash;
    _innerRadiusRatio = defaultModel._innerRadiusRatio;
    _points = defaultModel.points;
    notifyListeners();
  }
}

class ControlSlider extends StatelessWidget {
  const ControlSlider({
    super.key,
    required this.label,
    required this.value,
    required this.onChanged,
    this.min = 0.0,
    this.max = 1.0,
    this.precision = _kParameterPrecision,
  });

  final String label;
  final double value;
  final void Function(double value) onChanged;
  final double min;
  final double max;
  final int precision;

  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.all(4.0),
      child: Row(
        mainAxisSize: MainAxisSize.min,
        children: <Widget>[
          Expanded(flex: 2, child: Text(label, textAlign: TextAlign.end)),
          Expanded(flex: 5, child: Slider(onChanged: onChanged, min: min, max: max, value: value)),
          Expanded(child: Text(value.toStringAsFixed(precision))),
        ],
      ),
    );
  }
}
