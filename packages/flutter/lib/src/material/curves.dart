// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/animation.dart';

// The easing curves of the Material Library

/// The standard easing curve in the Material 2 specification.
///
/// Elements that begin and end at rest use standard easing.
/// They speed up quickly and slow down gradually, in order
/// to emphasize the end of the transition.
///
/// See also:
/// * <https://material.io/design/motion/speed.html#easing>
@Deprecated(
  'Use Easing.legacy (M2) or Easing.standard (M3) instead. '
  'This curve is updated in M3. '
  'This feature was deprecated after v3.18.0-0.1.pre.',
)
const Curve standardEasing = Curves.fastOutSlowIn;

/// The accelerate easing curve in the Material 2 specification.
///
/// Elements exiting a screen use acceleration easing,
/// where they start at rest and end at peak velocity.
///
/// See also:
/// * <https://material.io/design/motion/speed.html#easing>
@Deprecated(
  'Use Easing.legacyAccelerate (M2) or Easing.standardAccelerate (M3) instead. '
  'This curve is updated in M3. '
  'This feature was deprecated after v3.18.0-0.1.pre.',
)
const Curve accelerateEasing = Cubic(0.4, 0.0, 1.0, 1.0);

/// The decelerate easing curve in the Material 2 specification.
///
/// Incoming elements are animated using deceleration easing,
/// which starts a transition at peak velocity (the fastest
/// point of an element’s movement) and ends at rest.
///
/// See also:
/// * <https://material.io/design/motion/speed.html#easing>
@Deprecated(
  'Use Easing.legacyDecelerate (M2) or Easing.standardDecelerate (M3) instead. '
  'This curve is updated in M3. '
  'This feature was deprecated after v3.18.0-0.1.pre.',
)
const Curve decelerateEasing = Cubic(0.0, 0.0, 0.2, 1.0);
