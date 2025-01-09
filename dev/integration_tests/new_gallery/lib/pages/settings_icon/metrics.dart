// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math';
import 'package:flutter/material.dart';

// Color gradients.
const Color pinkLeft = Color(0xFFFF5983);
const Color pinkRight = Color(0xFFFF8383);

const Color tealLeft = Color(0xFF1CDDC8);
const Color tealRight = Color(0xFF00A5B3);

// Dimensions.
const int unitHeight = 1;
const int unitWidth = 1;

const double stickLength = 5 / 9;
const double stickWidth = 5 / 36;
const double stickRadius = stickWidth / 2;
const double knobDiameter = 5 / 54;
const double knobRadius = knobDiameter / 2;
const double stickGap = 5 / 54;

// Locations.
const double knobDistanceFromCenter = stickGap / 2 + stickWidth / 2;
const Offset lowerKnobCenter = Offset(0, knobDistanceFromCenter);
const Offset upperKnobCenter = Offset(0, -knobDistanceFromCenter);

const double knobDeviation = stickLength / 2 - stickRadius;

// Key moments in animation.
const double _colorKnobContractionBegins = 1 / 23;
const double _monoKnobExpansionEnds = 11 / 23;
const double _colorKnobContractionEnds = 14 / 23;

// Stages.
bool isTransitionPhase(double time) => time < _colorKnobContractionEnds;

// Curve easing.
const Cubic _curve = Curves.easeInOutCubic;

double _progress(double time, {required double begin, required double end}) =>
    _curve.transform(((time - begin) / (end - begin)).clamp(0, 1).toDouble());

double _monoKnobProgress(double time) => _progress(time, begin: 0, end: _monoKnobExpansionEnds);

double _colorKnobProgress(double time) =>
    _progress(time, begin: _colorKnobContractionBegins, end: _colorKnobContractionEnds);

double _rotationProgress(double time) => _progress(time, begin: _colorKnobContractionEnds, end: 1);

// Changing lengths: mono.
double monoLength(double time) =>
    _monoKnobProgress(time) * (stickLength - knobDiameter) + knobDiameter;

double _monoLengthLeft(double time) => min(monoLength(time) - knobRadius, stickRadius);

double _monoLengthRight(double time) => monoLength(time) - _monoLengthLeft(time);

double _monoHorizontalOffset(double time) =>
    (_monoLengthRight(time) - _monoLengthLeft(time)) / 2 - knobDeviation;

Offset upperMonoOffset(double time) => upperKnobCenter + Offset(_monoHorizontalOffset(time), 0);

Offset lowerMonoOffset(double time) => lowerKnobCenter + Offset(-_monoHorizontalOffset(time), 0);

// Changing lengths: color.
double colorLength(double time) => (1 - _colorKnobProgress(time)) * stickLength;

Offset upperColorOffset(double time) =>
    upperKnobCenter + Offset(stickLength / 2 - colorLength(time) / 2, 0);

Offset lowerColorOffset(double time) =>
    lowerKnobCenter + Offset(-stickLength / 2 + colorLength(time) / 2, 0);

// Moving objects.
double knobRotation(double time) => _rotationProgress(time) * pi / 4;

Offset knobCenter(double time) {
  final double progress = _rotationProgress(time);
  if (progress == 0) {
    return lowerKnobCenter;
  } else if (progress == 1) {
    return upperKnobCenter;
  } else {
    // Calculates the current location.
    final Offset center = Offset(knobDistanceFromCenter / tan(pi / 8), 0);
    final double radius = (lowerKnobCenter - center).distance;
    final double angle = pi + (progress - 1 / 2) * pi / 4;
    return center + Offset.fromDirection(angle, radius);
  }
}
