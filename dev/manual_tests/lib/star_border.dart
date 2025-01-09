// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';

void main() => runApp(const MyApp());

enum LerpTarget { circle, roundedRect, rect, stadium, polygon, star }

class MyApp extends StatelessWidget {
  const MyApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(home: MyHomePage());
  }
}

class MyHomePage extends StatefulWidget {
  const MyHomePage({super.key});

  @override
  State<MyHomePage> createState() => _MyHomePageState();
}

class _MyHomePageState extends State<MyHomePage> {
  static final GlobalKey<ScaffoldState> scaffoldKey = GlobalKey<ScaffoldState>();
  final OptionModel _model = OptionModel();
  final TextEditingController textController = TextEditingController();

  @override
  void initState() {
    super.initState();
    _model.addListener(_modelChanged);
  }

  @override
  void dispose() {
    super.dispose();
    _model.removeListener(_modelChanged);
  }

  void _modelChanged() {
    setState(() {});
  }

  @override
  Widget build(BuildContext context) {
    return SafeArea(
      child: Scaffold(
        key: scaffoldKey,
        appBar: AppBar(title: const Text('Star Border'), backgroundColor: const Color(0xff323232)),
        body: Column(
          children: <Widget>[
            ColoredBox(color: Colors.grey.shade200, child: Options(_model)),
            Expanded(
              child: Row(
                mainAxisAlignment: MainAxisAlignment.spaceEvenly,
                children: <Widget>[
                  Container(
                    alignment: Alignment.center,
                    width: 300,
                    height: 200,
                    decoration: ShapeDecoration(
                      color: Colors.blue.shade100,
                      shape:
                          lerpBorder(
                            StarBorder.polygon(
                              side: const BorderSide(
                                strokeAlign: BorderSide.strokeAlignCenter,
                                width: 2,
                              ),
                              sides: _model.points,
                              pointRounding: _model.pointRounding,
                              rotation: _model.rotation,
                              squash: _model.squash,
                            ),
                            _model._lerpTarget,
                            _model._lerpAmount,
                            to: _model.lerpTo,
                          )!,
                    ),
                    child: const Text('Polygon'),
                  ),
                  Container(
                    alignment: Alignment.center,
                    width: 300,
                    height: 200,
                    decoration: ShapeDecoration(
                      color: Colors.blue.shade100,
                      shape:
                          lerpBorder(
                            StarBorder(
                              side: const BorderSide(
                                strokeAlign: BorderSide.strokeAlignCenter,
                                width: 2,
                              ),
                              points: _model.points,
                              innerRadiusRatio: _model.innerRadiusRatio,
                              pointRounding: _model.pointRounding,
                              valleyRounding: _model.valleyRounding,
                              rotation: _model.rotation,
                              squash: _model.squash,
                            ),
                            _model._lerpTarget,
                            _model._lerpAmount,
                            to: _model.lerpTo,
                          )!,
                    ),
                    child: const Text('Star'),
                  ),
                ],
              ),
            ),
          ],
        ),
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
      _innerRadiusRatio = clampDouble(value, 0.0001, double.infinity);
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

  double get lerpAmount => _lerpAmount;
  double _lerpAmount = 0.0;
  set lerpAmount(double value) {
    if (value != _lerpAmount) {
      _lerpAmount = value;
      notifyListeners();
    }
  }

  bool get lerpTo => _lerpTo;
  bool _lerpTo = true;
  set lerpTo(bool value) {
    if (_lerpTo != value) {
      _lerpTo = value;
      notifyListeners();
    }
  }

  LerpTarget get lerpTarget => _lerpTarget;
  LerpTarget _lerpTarget = LerpTarget.circle;
  set lerpTarget(LerpTarget value) {
    if (value != _lerpTarget) {
      _lerpTarget = value;
      notifyListeners();
    }
  }

  void reset() {
    final OptionModel defaultModel = OptionModel();
    _pointRounding = defaultModel.pointRounding;
    _valleyRounding = defaultModel.valleyRounding;
    _rotation = defaultModel.rotation;
    _squash = defaultModel.squash;
    _lerpAmount = defaultModel.lerpAmount;
    _lerpTo = defaultModel.lerpTo;
    _lerpTarget = defaultModel.lerpTarget;
    _innerRadiusRatio = defaultModel._innerRadiusRatio;
    _points = defaultModel.points;
    notifyListeners();
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

  double sliderValue = 0.0;

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
                        min: 2,
                        max: 20,
                        onChanged: (double value) {
                          widget.model.points = value;
                        },
                      ),
                    ),
                    OutlinedButton(
                      child: const Text('Nearest'),
                      onPressed: () {
                        widget.model.points = widget.model.points.roundToDouble();
                      },
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
          Row(
            children: <Widget>[
              Expanded(
                flex: 2,
                child: Padding(
                  padding: const EdgeInsetsDirectional.only(end: 8.0),
                  child: ControlSlider(
                    label: 'Lerp',
                    value: widget.model.lerpAmount,
                    onChanged: (double value) {
                      widget.model.lerpAmount = value;
                    },
                  ),
                ),
              ),
              Padding(
                padding: const EdgeInsetsDirectional.only(start: 8.0, end: 20.0),
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: <Widget>[
                    Row(
                      children: <Widget>[
                        Radio<bool>(
                          value: true,
                          groupValue: widget.model.lerpTo,
                          onChanged: (bool? value) {
                            widget.model.lerpTo = value!;
                          },
                        ),
                        const Text('To'),
                      ],
                    ),
                    Row(
                      children: <Widget>[
                        Radio<bool>(
                          value: false,
                          groupValue: widget.model.lerpTo,
                          onChanged: (bool? value) {
                            widget.model.lerpTo = value!;
                          },
                        ),
                        const Text('From'),
                      ],
                    ),
                  ],
                ),
              ),
              Expanded(
                child: Row(
                  children: <Widget>[
                    Expanded(
                      child: DropdownButton<LerpTarget>(
                        items:
                            LerpTarget.values.map<DropdownMenuItem<LerpTarget>>((
                              LerpTarget target,
                            ) {
                              return DropdownMenuItem<LerpTarget>(
                                value: target,
                                child: Text(target.name),
                              );
                            }).toList(),
                        value: widget.model.lerpTarget,
                        onChanged: (LerpTarget? value) {
                          if (value == null) {
                            return;
                          }
                          widget.model.lerpTarget = value;
                        },
                      ),
                    ),
                  ],
                ),
              ),
            ],
          ),
          ElevatedButton(
            onPressed: () {
              widget.model.reset();
              sliderValue = 0.0;
            },
            child: const Text('Reset'),
          ),
        ],
      ),
    );
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
  });

  final String label;
  final double value;
  final void Function(double value) onChanged;
  final double min;
  final double max;

  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.all(4.0),
      child: Row(
        mainAxisSize: MainAxisSize.min,
        children: <Widget>[
          Text(label),
          Expanded(
            child: Slider(
              label: value.toStringAsFixed(1),
              onChanged: onChanged,
              min: min,
              max: max,
              value: value,
            ),
          ),
          Text(value.toStringAsFixed(3)),
        ],
      ),
    );
  }
}

const Color lerpToColor = Colors.red;
const BorderSide lerpToBorder = BorderSide(width: 5, color: lerpToColor);

ShapeBorder? lerpBorder(StarBorder border, LerpTarget target, double t, {bool to = true}) {
  final OutlinedBorder targetBorder = switch (target) {
    LerpTarget.circle => const CircleBorder(side: lerpToBorder, eccentricity: 0.5),
    LerpTarget.rect => const RoundedRectangleBorder(side: lerpToBorder),
    LerpTarget.stadium => const StadiumBorder(side: lerpToBorder),
    LerpTarget.polygon => const StarBorder.polygon(side: lerpToBorder, sides: 4),
    LerpTarget.star => const StarBorder(side: lerpToBorder, innerRadiusRatio: 0.5),
    LerpTarget.roundedRect => RoundedRectangleBorder(
      side: lerpToBorder,
      borderRadius: BorderRadius.circular(10),
    ),
  };
  return to ? border.lerpTo(targetBorder, t) : border.lerpFrom(targetBorder, t);
}
