// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This file is run as part of a reduced test set in CI on Mac and Windows
// machines.
@Tags(<String>['reduced-test-set'])
@TestOn('!chrome')
library;

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

import '../impeller_test_helpers.dart';

void main() {
  testWidgets('Color filter - red', (WidgetTester tester) async {
    await tester.pumpWidget(
      const RepaintBoundary(
        child: ColorFiltered(
          colorFilter: ColorFilter.mode(Colors.red, BlendMode.color),
          child: Placeholder(),
        ),
      ),
    );
    await expectLater(find.byType(ColorFiltered), matchesGoldenFile('color_filter_red.png'));
  });

  testWidgets('Color filter - sepia', (WidgetTester tester) async {
    const ColorFilter sepia = ColorFilter.matrix(<double>[
      0.39, 0.769, 0.189, 0, 0, //
      0.349, 0.686, 0.168, 0, 0, //
      0.272, 0.534, 0.131, 0, 0, //
      0, 0, 0, 1, 0, //
    ]);
    await tester.pumpWidget(
      RepaintBoundary(
        child: ColorFiltered(
          colorFilter: sepia,
          child: MaterialApp(
            debugShowCheckedModeBanner: false, // https://github.com/flutter/flutter/issues/143616
            title: 'Flutter Demo',
            theme: ThemeData(primarySwatch: Colors.blue, useMaterial3: false),
            home: Scaffold(
              appBar: AppBar(title: const Text('Sepia ColorFilter Test')),
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
    await expectLater(find.byType(ColorFiltered), matchesGoldenFile('color_filter_sepia.png'));
  }, skip: impellerEnabled); // https://github.com/flutter/flutter/issues/143616

  testWidgets('Color filter - reuses its layer', (WidgetTester tester) async {
    Future<void> pumpWithColor(Color color) async {
      await tester.pumpWidget(
        RepaintBoundary(
          child: ColorFiltered(
            colorFilter: ColorFilter.mode(color, BlendMode.color),
            child: const Placeholder(),
          ),
        ),
      );
    }

    await pumpWithColor(Colors.red);
    final RenderObject renderObject = tester.firstRenderObject(find.byType(ColorFiltered));
    final ColorFilterLayer originalLayer = renderObject.debugLayer! as ColorFilterLayer;
    expect(originalLayer, isNotNull);

    // Change color to force a repaint.
    await pumpWithColor(Colors.green);
    expect(renderObject.debugLayer, same(originalLayer));
  });
}
