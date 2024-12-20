// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/services.dart';

import 'package:flutter_test/flutter_test.dart';

void main() {
  test('MouseTrackerAnnotation has correct toString', () {
    final MouseTrackerAnnotation annotation1 = MouseTrackerAnnotation(
      onEnter: (_) {},
      onExit: (_) {},
    );
    expect(
      annotation1.toString(),
      equals('MouseTrackerAnnotation#${shortHash(annotation1)}(callbacks: [enter, exit])'),
    );

    const MouseTrackerAnnotation annotation2 = MouseTrackerAnnotation();
    expect(
      annotation2.toString(),
      equals('MouseTrackerAnnotation#${shortHash(annotation2)}(callbacks: <none>)'),
    );

    final MouseTrackerAnnotation annotation3 = MouseTrackerAnnotation(
      onEnter: (_) {},
      cursor: SystemMouseCursors.grab,
    );
    expect(
      annotation3.toString(),
      equals(
        'MouseTrackerAnnotation#${shortHash(annotation3)}(callbacks: [enter], cursor: SystemMouseCursor(grab))',
      ),
    );
  });
}
