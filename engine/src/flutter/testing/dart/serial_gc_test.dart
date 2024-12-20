// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// FlutterTesterOptions=--enable-serial-gc

import 'package:test/test.dart';

int use(List<int> a) {
  return a[0];
}

void main() {
  test('Serial GC option test ', () async {
    const bool threw = false;
    for (int i = 0; i < 100; i++) {
      final List<int> a = <int>[100];
      use(a);
    }
    expect(threw, false);
  });
}
