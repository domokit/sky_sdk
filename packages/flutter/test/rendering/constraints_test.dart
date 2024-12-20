// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

import 'rendering_tester.dart';

void main() {
  TestRenderingFlutterBinding.ensureInitialized();

  test('RenderFractionallySizedBox constraints', () {
    RenderBox root, leaf, test;
    root = RenderPositionedBox(
      child: RenderConstrainedBox(
        additionalConstraints: BoxConstraints.tight(const Size(200.0, 200.0)),
        child:
            test = RenderFractionallySizedOverflowBox(
              widthFactor: 2.0,
              heightFactor: 0.5,
              child:
                  leaf = RenderConstrainedBox(additionalConstraints: const BoxConstraints.expand()),
            ),
      ),
    );
    layout(root);
    expect(root.size.width, equals(800.0));
    expect(root.size.height, equals(600.0));
    expect(test.size.width, equals(200.0));
    expect(test.size.height, equals(200.0));
    expect(leaf.size.width, equals(400.0));
    expect(leaf.size.height, equals(100.0));
  });

  test('BoxConstraints with NaN', () {
    String result;

    result = 'no exception';
    try {
      const BoxConstraints constraints = BoxConstraints(
        minWidth: double.nan,
        maxWidth: double.nan,
        minHeight: 2.0,
        maxHeight: double.nan,
      );
      assert(constraints.debugAssertIsValid());
    } on FlutterError catch (e) {
      result = '$e';
    }
    expect(
      result,
      equals(
        'BoxConstraints has NaN values in minWidth, maxWidth, and maxHeight.\n'
        'The offending constraints were:\n'
        '  BoxConstraints(NaN<=w<=NaN, 2.0<=h<=NaN; NOT NORMALIZED)',
      ),
    );

    result = 'no exception';
    try {
      const BoxConstraints constraints = BoxConstraints(minHeight: double.nan);
      assert(constraints.debugAssertIsValid());
    } on FlutterError catch (e) {
      result = '$e';
    }
    expect(
      result,
      equals(
        'BoxConstraints has a NaN value in minHeight.\n'
        'The offending constraints were:\n'
        '  BoxConstraints(0.0<=w<=Infinity, NaN<=h<=Infinity; NOT NORMALIZED)',
      ),
    );

    result = 'no exception';
    try {
      const BoxConstraints constraints = BoxConstraints(minHeight: double.nan, maxWidth: 0.0 / 0.0);
      assert(constraints.debugAssertIsValid());
    } on FlutterError catch (e) {
      result = '$e';
    }
    expect(
      result,
      equals(
        'BoxConstraints has NaN values in maxWidth and minHeight.\n'
        'The offending constraints were:\n'
        '  BoxConstraints(0.0<=w<=NaN, NaN<=h<=Infinity; NOT NORMALIZED)',
      ),
    );
  });
}
