// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:test/test.dart';

void main() {
  test('Message loop flushes microtasks between iterations', () async {
    final List<int> tasks = <int>[];

    tasks.add(1);

    // Flush 0 microtasks.
    await Future<void>.delayed(Duration.zero);

    scheduleMicrotask(() {
      tasks.add(3);
    });
    scheduleMicrotask(() {
      tasks.add(4);
    });

    tasks.add(2);

    // Flush 2 microtasks.
    await Future<void>.delayed(Duration.zero);

    scheduleMicrotask(() {
      tasks.add(6);
    });
    scheduleMicrotask(() {
      tasks.add(7);
    });
    scheduleMicrotask(() {
      tasks.add(8);
    });

    tasks.add(5);

    // Flush 3 microtasks.
    await Future<void>.delayed(Duration.zero);

    tasks.add(9);

    expect(tasks, <int>[1, 2, 3, 4, 5, 6, 7, 8, 9]);
  });
}
