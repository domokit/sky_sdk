// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui';

import 'package:flutter/material.dart';

class SlidersPage extends StatefulWidget {
  const SlidersPage({super.key});

  @override
  State<SlidersPage> createState() => _SlidersPageState();
}

class _SlidersPageState extends State<SlidersPage> with TickerProviderStateMixin {
  late AnimationController _sliderController;
  late Animation<double> _sliderAnimation;
  double _sliderValue = 0.0;
  RangeValues _rangeSliderValues = const RangeValues(0.0, 1.0);

  @override
  void initState() {
    super.initState();
    _sliderController = AnimationController(duration: const Duration(seconds: 1), vsync: this)
      ..repeat();
    _sliderAnimation = Tween<double>(begin: 0, end: 1).animate(_sliderController)..addListener(() {
      setState(() {
        _sliderValue = _sliderAnimation.value;
        _rangeSliderValues = RangeValues(
          clampDouble(_sliderAnimation.value, 0, 0.45),
          1.0 - clampDouble(_sliderAnimation.value, 0, 0.45),
        );
      });
    });
  }

  @override
  void dispose() {
    _sliderController.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Material(
      child: Column(
        mainAxisAlignment: MainAxisAlignment.center,
        children: <Widget>[
          Slider(value: _sliderValue, onChanged: (double value) {}),
          RangeSlider(values: _rangeSliderValues, onChanged: (RangeValues values) {}),
        ],
      ),
    );
  }
}
