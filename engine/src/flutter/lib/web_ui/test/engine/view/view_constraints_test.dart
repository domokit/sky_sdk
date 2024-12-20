// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' as ui;

import '../../common/matchers.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

Future<void> testMain() async {
  const ui.Size size = ui.Size(640, 480);

  group('ViewConstraints.fromJs', () {
    test('Negative min constraints -> Assertion error.', () async {
      expect(
        () => ViewConstraints.fromJs(JsViewConstraints(minWidth: -1), size),
        throwsAssertionError,
      );
      expect(
        () => ViewConstraints.fromJs(JsViewConstraints(minHeight: -1), size),
        throwsAssertionError,
      );
    });

    test('Infinite min constraints -> Assertion error.', () async {
      expect(
        () => ViewConstraints.fromJs(JsViewConstraints(minWidth: double.infinity), size),
        throwsAssertionError,
      );
      expect(
        () => ViewConstraints.fromJs(JsViewConstraints(minHeight: double.infinity), size),
        throwsAssertionError,
      );
    });

    test('Negative max constraints -> Assertion error.', () async {
      expect(
        () => ViewConstraints.fromJs(JsViewConstraints(maxWidth: -1), size),
        throwsAssertionError,
      );
      expect(
        () => ViewConstraints.fromJs(JsViewConstraints(maxHeight: -1), size),
        throwsAssertionError,
      );
    });

    test('null JS Constraints -> Tight to size', () async {
      expect(
        ViewConstraints.fromJs(null, size),
        const ViewConstraints(
          minWidth: 640,
          maxWidth: 640, //
          minHeight: 480,
          maxHeight: 480, //
        ),
      );
    });

    test('non-null JS Constraints -> Computes sizes', () async {
      final JsViewConstraints constraints = JsViewConstraints(
        minWidth: 500,
        maxWidth: 600, //
        minHeight: 300,
        maxHeight: 400, //
      );
      expect(
        ViewConstraints.fromJs(constraints, size),
        const ViewConstraints(
          minWidth: 500,
          maxWidth: 600, //
          minHeight: 300,
          maxHeight: 400, //
        ),
      );
    });

    test('null JS Width -> Tight to width. Computes height.', () async {
      final JsViewConstraints constraints = JsViewConstraints(minHeight: 200, maxHeight: 320);
      expect(
        ViewConstraints.fromJs(constraints, size),
        const ViewConstraints(
          minWidth: 640,
          maxWidth: 640, //
          minHeight: 200,
          maxHeight: 320, //
        ),
      );
    });

    test('null JS Height -> Tight to height. Computed width.', () async {
      final JsViewConstraints constraints = JsViewConstraints(minWidth: 200, maxWidth: 320);
      expect(
        ViewConstraints.fromJs(constraints, size),
        const ViewConstraints(
          minWidth: 200,
          maxWidth: 320, //
          minHeight: 480,
          maxHeight: 480, //
        ),
      );
    });

    test(
      'non-null JS Constraints -> Computes sizes. Max values can be greater than available size.',
      () async {
        final JsViewConstraints constraints = JsViewConstraints(
          minWidth: 500,
          maxWidth: 1024, //
          minHeight: 300,
          maxHeight: 768, //
        );
        expect(
          ViewConstraints.fromJs(constraints, size),
          const ViewConstraints(
            minWidth: 500,
            maxWidth: 1024, //
            minHeight: 300,
            maxHeight: 768, //
          ),
        );
      },
    );

    test('non-null JS Constraints -> Computes sizes. Max values can be unconstrained.', () async {
      final JsViewConstraints constraints = JsViewConstraints(
        minWidth: 500,
        maxWidth: double.infinity,
        minHeight: 300,
        maxHeight: double.infinity,
      );
      expect(
        ViewConstraints.fromJs(constraints, size),
        const ViewConstraints(
          minWidth: 500,
          // ignore: avoid_redundant_argument_values
          maxWidth: double.infinity,
          minHeight: 300,
          // ignore: avoid_redundant_argument_values
          maxHeight: double.infinity,
        ),
      );
    });
  });
}
