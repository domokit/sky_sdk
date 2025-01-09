// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import '../../foo/fake_render_box.dart';

mixin ARenderBoxMixin on RenderBox {
  @override
  void computeMaxIntrinsicWidth() {}

  @override
  void computeMinIntrinsicWidth() => computeMaxIntrinsicWidth(); // ERROR: computeMaxIntrinsicWidth(). Consider calling getMaxIntrinsicWidth instead.

  @override
  void computeMinIntrinsicHeight() {
    final void Function() f =
        computeMaxIntrinsicWidth; // ERROR: f = computeMaxIntrinsicWidth. Consider calling getMaxIntrinsicWidth instead.
    f();
  }
}

extension ARenderBoxExtension on RenderBox {
  void test() {
    computeDryBaseline(); // ERROR: computeDryBaseline(). Consider calling getDryBaseline instead.
    computeDryLayout(); // ERROR: computeDryLayout(). Consider calling getDryLayout instead.
  }
}

class RenderBoxSubclass1 extends RenderBox {
  @override
  void computeDryLayout() {
    computeDistanceToActualBaseline(); // ERROR: computeDistanceToActualBaseline(). Consider calling getDistanceToBaseline, or getDistanceToActualBaseline instead.
  }

  @override
  void computeDistanceToActualBaseline() {
    computeMaxIntrinsicHeight(); // ERROR: computeMaxIntrinsicHeight(). Consider calling getMaxIntrinsicHeight instead.
  }
}

class RenderBoxSubclass2 extends RenderBox with ARenderBoxMixin {
  @override
  void computeMaxIntrinsicWidth() {
    super.computeMinIntrinsicHeight(); // OK
    super.computeMaxIntrinsicWidth(); // OK
    final void Function() f = super.computeDryBaseline; // OK
    f();
  }
}
