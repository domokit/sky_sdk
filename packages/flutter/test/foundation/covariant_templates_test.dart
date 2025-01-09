// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_test/flutter_test.dart';

class X {}

class Y extends X {}

class A<U extends X> {
  U? u;
}

void main() {
  test('Assignment through a covariant template throws exception', () {
    final A<Y> ay = A<Y>();
    final A<X> ayAsAx = ay;
    expect(() {
      ayAsAx.u = X();
    }, throwsA(isA<TypeError>()));
  });
}
