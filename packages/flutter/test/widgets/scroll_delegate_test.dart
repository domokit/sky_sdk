// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:leak_tracker_flutter_testing/leak_tracker_flutter_testing.dart';

void main() {
  test('TwoDimensionalChildBuilderDelegate dispatches memory events', () async {
    await expectLater(
      await memoryEvents(
        () => TwoDimensionalChildBuilderDelegate(builder: (_, _) => null).dispose(),
        TwoDimensionalChildBuilderDelegate,
      ),
      areCreateAndDispose,
    );
  });

  test('TwoDimensionalChildListDelegate dispatches memory events', () async {
    await expectLater(
      await memoryEvents(
        () => TwoDimensionalChildListDelegate(children: <List<Widget>>[]).dispose(),
        TwoDimensionalChildListDelegate,
      ),
      areCreateAndDispose,
    );
  });
}
