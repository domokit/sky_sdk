// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This file is run as part of a reduced test set in CI on Mac and Windows
// machines.
@Tags(<String>['reduced-test-set'])
library;

import 'dart:math';
import 'dart:ui';

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Image filter - blur', (WidgetTester tester) async {
    await tester.pumpWidget(
      RepaintBoundary(
        child: ImageFiltered(
          imageFilter: ImageFilter.blur(sigmaX: 10.0, sigmaY: 10.0),
          child: const Placeholder(),
        ),
      ),
    );
    await expectLater(find.byType(ImageFiltered), matchesGoldenFile('image_filter_blur.png'));
  });

  testWidgets('Image filter - blur with offset', (WidgetTester tester) async {
    final Key key = GlobalKey();
    await tester.pumpWidget(
      RepaintBoundary(
        key: key,
        child: Transform.translate(
          offset: const Offset(50, 50),
          child: ImageFiltered(
            imageFilter: ImageFilter.blur(sigmaX: 3.0, sigmaY: 3.0),
            child: const Placeholder(),
          ),
        ),
      ),
    );
    await expectLater(find.byKey(key), matchesGoldenFile('image_filter_blur_offset.png'));
  });

  testWidgets('Image filter - dilate', (WidgetTester tester) async {
    await tester.pumpWidget(
      RepaintBoundary(
        child: ImageFiltered(
          imageFilter: ImageFilter.dilate(radiusX: 10.0, radiusY: 10.0),
          child: const Placeholder(),
        ),
      ),
    );
    await expectLater(find.byType(ImageFiltered), matchesGoldenFile('image_filter_dilate.png'));
  }, skip: kIsWeb); // https://github.com/flutter/flutter/issues/101874

  testWidgets('Image filter - erode', (WidgetTester tester) async {
    await tester.pumpWidget(
      RepaintBoundary(
        child: ImageFiltered(
          // Do not erode too much, otherwise we will see nothing left.
          imageFilter: ImageFilter.erode(radiusX: 1.0, radiusY: 1.0),
          child: const Placeholder(strokeWidth: 4),
        ),
      ),
    );
    await expectLater(find.byType(ImageFiltered), matchesGoldenFile('image_filter_erode.png'));
  }, skip: kIsWeb); // https://github.com/flutter/flutter/issues/101874

  testWidgets('Image filter - matrix', (WidgetTester tester) async {
    final ImageFilter matrix = ImageFilter.matrix(
      Float64List.fromList(<double>[
        0.5, 0.0, 0.0, 0.0, //
        0.0, 0.5, 0.0, 0.0, //
        0.0, 0.0, 1.0, 0.0, //
        0.0, 0.0, 0.0, 1.0, //
      ]),
    );
    await tester.pumpWidget(
      RepaintBoundary(
        child: ImageFiltered(
          imageFilter: matrix,
          child: MaterialApp(
            title: 'Flutter Demo',
            theme: ThemeData(useMaterial3: false, primarySwatch: Colors.blue),
            debugShowCheckedModeBanner: false, // https://github.com/flutter/flutter/issues/143616
            home: Scaffold(
              appBar: AppBar(title: const Text('Matrix ImageFilter Test')),
              body: const Center(child: Text('Hooray!')),
              floatingActionButton: FloatingActionButton(
                onPressed: () {},
                tooltip: 'Increment',
                child: const Icon(Icons.add),
              ),
            ),
          ),
        ),
      ),
    );
    await expectLater(find.byType(ImageFiltered), matchesGoldenFile('image_filter_matrix.png'));
  });

  testWidgets('Image filter - matrix with offset', (WidgetTester tester) async {
    final Matrix4 matrix = Matrix4.rotationZ(pi / 18);
    final ImageFilter matrixFilter = ImageFilter.matrix(matrix.storage);
    final Key key = GlobalKey();
    await tester.pumpWidget(
      RepaintBoundary(
        key: key,
        child: Transform.translate(
          offset: const Offset(50, 50),
          child: ImageFiltered(
            imageFilter: matrixFilter,
            child: MaterialApp(
              title: 'Flutter Demo',
              theme: ThemeData(useMaterial3: false, primarySwatch: Colors.blue),
              debugShowCheckedModeBanner: false, // https://github.com/flutter/flutter/issues/143616
              home: Scaffold(
                appBar: AppBar(title: const Text('Matrix ImageFilter Test')),
                body: const Center(child: Text('Hooray!')),
                floatingActionButton: FloatingActionButton(
                  onPressed: () {},
                  tooltip: 'Increment',
                  child: const Icon(Icons.add),
                ),
              ),
            ),
          ),
        ),
      ),
    );
    await expectLater(find.byKey(key), matchesGoldenFile('image_filter_matrix_offset.png'));
  });

  testWidgets('Image filter - reuses its layer', (WidgetTester tester) async {
    Future<void> pumpWithSigma(double sigma) async {
      await tester.pumpWidget(
        RepaintBoundary(
          child: ImageFiltered(
            imageFilter: ImageFilter.blur(sigmaX: sigma, sigmaY: sigma),
            child: const Placeholder(),
          ),
        ),
      );
    }

    await pumpWithSigma(5.0);
    final RenderObject renderObject = tester.firstRenderObject(find.byType(ImageFiltered));
    final ImageFilterLayer originalLayer = renderObject.debugLayer! as ImageFilterLayer;

    // Change blur sigma to force a repaint.
    await pumpWithSigma(10.0);
    expect(renderObject.debugLayer, same(originalLayer));
  });

  testWidgets('Image filter - enabled and disabled', (WidgetTester tester) async {
    Future<void> pumpWithEnabledState(bool enabled) async {
      await tester.pumpWidget(
        RepaintBoundary(
          child: ImageFiltered(
            enabled: enabled,
            imageFilter: ImageFilter.blur(sigmaX: 5, sigmaY: 5),
            child: const Placeholder(),
          ),
        ),
      );
    }

    await pumpWithEnabledState(false);
    expect(tester.layers, isNot(contains(isA<ImageFilterLayer>())));

    await pumpWithEnabledState(true);
    expect(tester.layers, contains(isA<ImageFilterLayer>()));
  });
}
