// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:typed_data';
import 'dart:ui' as ui;

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

import '../impeller_test_helpers.dart';

const double _crispText = 100.0; // this font size is selected to avoid needing any antialiasing.
const String _expText = 'Éxp'; // renders in the test font as:

// ########
// ########
// ########
//     ########
//
// ÉÉÉÉxxxxpppp

void main() {
  testWidgets(
    'Default background',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        const Align(
          alignment: Alignment.topLeft,
          child: Text(
            _expText,
            textDirection: TextDirection.ltr,
            style: TextStyle(color: Color(0xFF345678), fontSize: _crispText),
          ),
        ),
      );
      await _expectColors(
        tester,
        find.byType(Align),
        <Color>{const Color(0x00000000), const Color(0xFF345678)},
        <Offset, Color>{
          Offset.zero: const Color(0xFF345678), // the text
          const Offset(10, 10): const Color(0xFF345678), // the text
          const Offset(50, 95): const Color(0x00000000), // the background (under the É)
          const Offset(250, 50): const Color(0x00000000), // the text (above the p)
          const Offset(250, 95): const Color(0xFF345678), // the text (the p)
          const Offset(400, 400): const Color(0x00000000), // the background
          const Offset(799, 599): const Color(0x00000000), // the background
        },
      );
    },
    // [intended] Test relies on captureImage, which is not supported on web currently.
    skip: !canCaptureImage || impellerEnabled,
  );

  testWidgets(
    'Default text color',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        const ColoredBox(
          color: Color(0xFFABCDEF),
          child: Align(
            alignment: Alignment.topLeft,
            child: Text(
              'Éxp',
              textDirection: TextDirection.ltr,
              style: TextStyle(fontSize: _crispText),
            ),
          ),
        ),
      );
      await _expectColors(
        tester,
        find.byType(Align),
        <Color>{const Color(0xFFABCDEF), const Color(0xFFFFFFFF)},
        <Offset, Color>{
          Offset.zero: const Color(0xFFFFFFFF), // the text
          const Offset(10, 10): const Color(0xFFFFFFFF), // the text
          const Offset(50, 95): const Color(0xFFABCDEF), // the background (under the É)
          const Offset(250, 50): const Color(0xFFABCDEF), // the text (above the p)
          const Offset(250, 95): const Color(0xFFFFFFFF), // the text (the p)
          const Offset(400, 400): const Color(0xFFABCDEF), // the background
          const Offset(799, 599): const Color(0xFFABCDEF), // the background
        },
      );
    },
    // [intended] Test relies on captureImage, which is not supported on web currently.
    skip: !canCaptureImage || impellerEnabled,
  );

  testWidgets(
    'Default text selection color',
    (WidgetTester tester) async {
      final GlobalKey key = GlobalKey();
      final OverlayEntry overlayEntry = OverlayEntry(
        builder:
            (BuildContext context) => SelectableRegion(
              selectionControls: emptyTextSelectionControls,
              child: Align(
                key: key,
                alignment: Alignment.topLeft,
                child: const Text(
                  'Éxp',
                  textDirection: TextDirection.ltr,
                  style: TextStyle(fontSize: _crispText, color: Color(0xFF000000)),
                ),
              ),
            ),
      );
      addTearDown(
        () =>
            overlayEntry
              ..remove()
              ..dispose(),
      );
      await tester.pumpWidget(
        ColoredBox(
          color: const Color(0xFFFFFFFF),
          child: Directionality(
            textDirection: TextDirection.ltr,
            child: MediaQuery(
              data: const MediaQueryData(),
              child: Overlay(initialEntries: <OverlayEntry>[overlayEntry]),
            ),
          ),
        ),
      );
      await tester.pumpAndSettle();
      await _expectColors(tester, find.byType(Align), <Color>{
        const Color(0xFFFFFFFF),
        const Color(0xFF000000),
      });
      // fake a "select all" event to select the text
      Actions.invoke(
        key.currentContext!,
        const SelectAllTextIntent(SelectionChangedCause.keyboard),
      );
      await tester.pump();
      await _expectColors(
        tester,
        find.byType(Align),
        <Color>{
          const Color(0xFFFFFFFF),
          const Color(0xFF000000),
          const Color(0xFFBFBFBF),
        }, // 0x80808080 blended with 0xFFFFFFFF
        <Offset, Color>{
          Offset.zero: const Color(0xFF000000), // the selected text
          const Offset(10, 10): const Color(0xFF000000), // the selected text
          const Offset(50, 95): const Color(0xFFBFBFBF), // the selected background (under the É)
          const Offset(250, 50): const Color(0xFFBFBFBF), // the selected background (above the p)
          const Offset(250, 95): const Color(0xFF000000), // the selected text (the p)
          const Offset(400, 400): const Color(0xFFFFFFFF), // the background
          const Offset(799, 599): const Color(0xFFFFFFFF), // the background
        },
      );
    },
    // [intended] Test relies on captureImage, which is not supported on web currently.
    skip: !canCaptureImage || impellerEnabled,
  );
}

Color _getPixel(ByteData bytes, int x, int y, int width) {
  final int offset = (x + y * width) * 4;
  return Color.fromARGB(
    bytes.getUint8(offset + 3),
    bytes.getUint8(offset + 0),
    bytes.getUint8(offset + 1),
    bytes.getUint8(offset + 2),
  );
}

Future<void> _expectColors(
  WidgetTester tester,
  Finder finder,
  Set<Color> allowedColors, [
  Map<Offset, Color>? spotChecks,
]) async {
  final TestWidgetsFlutterBinding binding = tester.binding;
  final ui.Image image =
      (await binding.runAsync<ui.Image>(() => captureImage(finder.evaluate().single)))!;
  addTearDown(image.dispose);
  final ByteData bytes =
      (await binding.runAsync<ByteData?>(
        () => image.toByteData(format: ui.ImageByteFormat.rawStraightRgba),
      ))!;
  final Set<int> actualColorValues = <int>{};
  for (int offset = 0; offset < bytes.lengthInBytes; offset += 4) {
    actualColorValues.add(
      (bytes.getUint8(offset + 3) << 24) +
          (bytes.getUint8(offset + 0) << 16) +
          (bytes.getUint8(offset + 1) << 8) +
          (bytes.getUint8(offset + 2)),
    );
  }
  final Set<Color> actualColors = actualColorValues.map((int value) => Color(value)).toSet();
  expect(actualColors, allowedColors);
  spotChecks?.forEach((Offset position, Color expected) {
    assert(position.dx.round() >= 0);
    assert(position.dx.round() < image.width);
    assert(position.dy.round() >= 0);
    assert(position.dy.round() < image.height);
    final Offset precisePosition = position * tester.view.devicePixelRatio;
    final Color actual = _getPixel(
      bytes,
      precisePosition.dx.round(),
      precisePosition.dy.round(),
      image.width,
    );
    expect(actual, expected, reason: 'Pixel at $position is $actual but expected $expected.');
  });
}
