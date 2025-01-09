// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This file is run as part of a reduced test set in CI on Mac and Windows
// machines.
@Tags(<String>['reduced-test-set'])
library;

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Slider value indicator', (WidgetTester tester) async {
    await _buildValueIndicatorStaticSlider(tester, value: 0, useMaterial3: true);

    await _pressStartThumb(tester);

    await expectLater(
      find.byType(MaterialApp),
      matchesGoldenFile('slider_m3_start_text_scale_1_width_0.png'),
    );

    await _buildValueIndicatorStaticSlider(tester, value: 0.5, useMaterial3: true);

    await _pressMiddleThumb(tester);

    await expectLater(
      find.byType(MaterialApp),
      matchesGoldenFile('slider_m3_middle_text_scale_1_width_0.png'),
    );

    await _buildValueIndicatorStaticSlider(tester, value: 1, useMaterial3: true);

    await _pressEndThumb(tester);

    await expectLater(
      find.byType(MaterialApp),
      matchesGoldenFile('slider_m3_end_text_scale_1_width_0.png'),
    );
  });

  testWidgets('Slider value indicator wide text', (WidgetTester tester) async {
    await _buildValueIndicatorStaticSlider(tester, value: 0, decimalCount: 5, useMaterial3: true);

    await _pressStartThumb(tester);

    await expectLater(
      find.byType(MaterialApp),
      matchesGoldenFile('slider_m3_start_text_scale_1_width_5.png'),
    );

    await _buildValueIndicatorStaticSlider(tester, value: 0.5, decimalCount: 5, useMaterial3: true);

    await _pressMiddleThumb(tester);

    await expectLater(
      find.byType(MaterialApp),
      matchesGoldenFile('slider_m3_middle_text_scale_1_width_5.png'),
    );

    await _buildValueIndicatorStaticSlider(tester, value: 1, decimalCount: 5, useMaterial3: true);

    await _pressEndThumb(tester);

    await expectLater(
      find.byType(MaterialApp),
      matchesGoldenFile('slider_m3_end_text_scale_1_width_5.png'),
    );
  });

  testWidgets('Slider value indicator large text scale', (WidgetTester tester) async {
    await _buildValueIndicatorStaticSlider(tester, value: 0, textScale: 3, useMaterial3: true);

    await _pressStartThumb(tester);

    await expectLater(
      find.byType(MaterialApp),
      matchesGoldenFile('slider_m3_start_text_scale_4_width_0.png'),
    );

    await _buildValueIndicatorStaticSlider(tester, value: 0.5, textScale: 3, useMaterial3: true);

    await _pressMiddleThumb(tester);

    await expectLater(
      find.byType(MaterialApp),
      matchesGoldenFile('slider_m3_middle_text_scale_4_width_0.png'),
    );

    await _buildValueIndicatorStaticSlider(tester, value: 1, textScale: 3, useMaterial3: true);

    await _pressEndThumb(tester);

    await expectLater(
      find.byType(MaterialApp),
      matchesGoldenFile('slider_m3_end_text_scale_4_width_0.png'),
    );
  });

  testWidgets('Slider value indicator large text scale and wide text', (WidgetTester tester) async {
    await _buildValueIndicatorStaticSlider(
      tester,
      value: 0,
      textScale: 3,
      decimalCount: 5,
      useMaterial3: true,
    );

    await _pressStartThumb(tester);

    await expectLater(
      find.byType(MaterialApp),
      matchesGoldenFile('slider_m3_start_text_scale_4_width_5.png'),
    );

    await _buildValueIndicatorStaticSlider(
      tester,
      value: 0.5,
      textScale: 3,
      decimalCount: 5,
      useMaterial3: true,
    );

    await _pressMiddleThumb(tester);

    await expectLater(
      find.byType(MaterialApp),
      matchesGoldenFile('slider_m3_middle_text_scale_4_width_5.png'),
    );

    await _buildValueIndicatorStaticSlider(
      tester,
      value: 1,
      textScale: 3,
      decimalCount: 5,
      useMaterial3: true,
    );

    await _pressEndThumb(tester);

    await expectLater(
      find.byType(MaterialApp),
      matchesGoldenFile('slider_m3_end_text_scale_4_width_5.png'),
    );
  });

  group('Material 2', () {
    // These tests are only relevant for Material 2. Once Material 2
    // support is deprecated and the APIs are removed, these tests
    // can be deleted.

    testWidgets('Slider value indicator', (WidgetTester tester) async {
      await _buildValueIndicatorStaticSlider(tester, value: 0);

      await _pressStartThumb(tester);

      await expectLater(
        find.byType(MaterialApp),
        matchesGoldenFile('slider_start_text_scale_1_width_0.png'),
      );

      await _buildValueIndicatorStaticSlider(tester, value: 0.5);

      await _pressMiddleThumb(tester);

      await expectLater(
        find.byType(MaterialApp),
        matchesGoldenFile('slider_middle_text_scale_1_width_0.png'),
      );

      await _buildValueIndicatorStaticSlider(tester, value: 1);

      await _pressEndThumb(tester);

      await expectLater(
        find.byType(MaterialApp),
        matchesGoldenFile('slider_end_text_scale_1_width_0.png'),
      );
    });

    testWidgets('Slider value indicator wide text', (WidgetTester tester) async {
      await _buildValueIndicatorStaticSlider(tester, value: 0, decimalCount: 5);

      await _pressStartThumb(tester);

      await expectLater(
        find.byType(MaterialApp),
        matchesGoldenFile('slider_start_text_scale_1_width_5.png'),
      );

      await _buildValueIndicatorStaticSlider(tester, value: 0.5, decimalCount: 5);

      await _pressMiddleThumb(tester);

      await expectLater(
        find.byType(MaterialApp),
        matchesGoldenFile('slider_middle_text_scale_1_width_5.png'),
      );

      await _buildValueIndicatorStaticSlider(tester, value: 1, decimalCount: 5);

      await _pressEndThumb(tester);

      await expectLater(
        find.byType(MaterialApp),
        matchesGoldenFile('slider_end_text_scale_1_width_5.png'),
      );
    });

    testWidgets('Slider value indicator large text scale', (WidgetTester tester) async {
      await _buildValueIndicatorStaticSlider(tester, value: 0, textScale: 3);

      await _pressStartThumb(tester);

      await expectLater(
        find.byType(MaterialApp),
        matchesGoldenFile('slider_start_text_scale_4_width_0.png'),
      );

      await _buildValueIndicatorStaticSlider(tester, value: 0.5, textScale: 3);

      await _pressMiddleThumb(tester);

      await expectLater(
        find.byType(MaterialApp),
        matchesGoldenFile('slider_middle_text_scale_4_width_0.png'),
      );

      await _buildValueIndicatorStaticSlider(tester, value: 1, textScale: 3);

      await _pressEndThumb(tester);

      await expectLater(
        find.byType(MaterialApp),
        matchesGoldenFile('slider_end_text_scale_4_width_0.png'),
      );
    });

    testWidgets('Slider value indicator large text scale and wide text', (
      WidgetTester tester,
    ) async {
      await _buildValueIndicatorStaticSlider(tester, value: 0, textScale: 3, decimalCount: 5);

      await _pressStartThumb(tester);

      await expectLater(
        find.byType(MaterialApp),
        matchesGoldenFile('slider_start_text_scale_4_width_5.png'),
      );

      await _buildValueIndicatorStaticSlider(tester, value: 0.5, textScale: 3, decimalCount: 5);

      await _pressMiddleThumb(tester);

      await expectLater(
        find.byType(MaterialApp),
        matchesGoldenFile('slider_middle_text_scale_4_width_5.png'),
      );

      await _buildValueIndicatorStaticSlider(tester, value: 1, textScale: 3, decimalCount: 5);

      await _pressEndThumb(tester);

      await expectLater(
        find.byType(MaterialApp),
        matchesGoldenFile('slider_end_text_scale_4_width_5.png'),
      );
    });
  });
}

Future<void> _pressStartThumb(WidgetTester tester) async {
  final Offset bottomLeft = tester.getBottomLeft(find.byType(Slider));
  final Offset topLeft = tester.getTopLeft(find.byType(Slider));
  final Offset left = (bottomLeft + topLeft) / 2;
  final Offset start = left + const Offset(24, 0);
  final TestGesture gesture = await tester.startGesture(start);
  await tester.pumpAndSettle();

  addTearDown(() async {
    // Finish gesture to release resources.
    await gesture.up();
    await tester.pumpAndSettle();
  });
}

Future<void> _pressMiddleThumb(WidgetTester tester) async {
  await tester.press(find.byType(Slider));
  await tester.pumpAndSettle();
}

Future<void> _pressEndThumb(WidgetTester tester) async {
  final Offset bottomRight = tester.getBottomRight(find.byType(Slider));
  final Offset topRight = tester.getTopRight(find.byType(Slider));
  final Offset right = (bottomRight + topRight) / 2;
  final Offset start = right - const Offset(24, 0);
  final TestGesture gesture = await tester.startGesture(start);
  await tester.pumpAndSettle();

  addTearDown(() async {
    // Finish gesture to release resources.
    await gesture.up();
    await tester.pumpAndSettle();
  });
}

Future<void> _buildValueIndicatorStaticSlider(
  WidgetTester tester, {
  required double value,
  double textScale = 1.0,
  int decimalCount = 0,
  bool useMaterial3 = false,
}) async {
  await tester.pumpWidget(
    MaterialApp(
      debugShowCheckedModeBanner: false, // https://github.com/flutter/flutter/issues/143616
      theme: ThemeData(useMaterial3: useMaterial3),
      home: Scaffold(
        body: Builder(
          builder: (BuildContext context) {
            return Center(
              child: MediaQuery.withClampedTextScaling(
                minScaleFactor: textScale,
                maxScaleFactor: textScale,
                child: SliderTheme(
                  data: Theme.of(
                    context,
                  ).sliderTheme.copyWith(showValueIndicator: ShowValueIndicator.always),
                  child: Slider(
                    value: value,
                    label: value.toStringAsFixed(decimalCount),
                    onChanged: (double newValue) {},
                  ),
                ),
              ),
            );
          },
        ),
      ),
    ),
  );
}
