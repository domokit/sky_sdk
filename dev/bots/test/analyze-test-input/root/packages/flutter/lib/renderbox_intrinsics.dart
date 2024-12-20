// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import '../../foo/fake_render_box.dart';

mixin ARenderBoxMixin on RenderBox {
  @override
  void computeMaxIntrinsicWidth() {}

  @override
  void computeMinIntrinsicWidth() => computeMaxIntrinsicWidth(); // BAD

  @override
  void computeMinIntrinsicHeight() {
    final void Function() f = computeMaxIntrinsicWidth; // BAD
    f();
  }
}

extension ARenderBoxExtension on RenderBox {
  void test() {
    computeDryBaseline(); // BAD
    computeDryLayout(); // BAD
  }
}

class RenderBoxSubclass1 extends RenderBox {
  @override
  void computeDryLayout() {
    computeDistanceToActualBaseline(); // BAD
  }

  @override
  void computeDistanceToActualBaseline() {
    computeMaxIntrinsicHeight(); // BAD
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
