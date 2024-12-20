// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/gestures.dart' show kPressTimeout;
import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

bool confirmCalled = false;
bool cancelCalled = false;

class TestInkSplash extends InkSplash {
  TestInkSplash({
    required super.controller,
    required super.referenceBox,
    super.position,
    required super.color,
    super.containedInkWell,
    super.rectCallback,
    super.borderRadius,
    super.customBorder,
    super.radius,
    super.onRemoved,
    required super.textDirection,
  });

  @override
  void confirm() {
    confirmCalled = true;
    super.confirm();
  }

  @override
  void cancel() {
    cancelCalled = true;
    super.cancel();
  }
}

class TestInkSplashFactory extends InteractiveInkFeatureFactory {
  const TestInkSplashFactory();

  @override
  InteractiveInkFeature create({
    required MaterialInkController controller,
    required RenderBox referenceBox,
    Offset? position,
    required Color color,
    bool containedInkWell = false,
    RectCallback? rectCallback,
    BorderRadius? borderRadius,
    ShapeBorder? customBorder,
    double? radius,
    VoidCallback? onRemoved,
    required TextDirection textDirection,
  }) {
    return TestInkSplash(
      controller: controller,
      referenceBox: referenceBox,
      position: position,
      color: color,
      containedInkWell: containedInkWell,
      rectCallback: rectCallback,
      borderRadius: borderRadius,
      customBorder: customBorder,
      radius: radius,
      onRemoved: onRemoved,
      textDirection: textDirection,
    );
  }
}

void main() {
  setUp(() {
    confirmCalled = false;
    cancelCalled = false;
  });

  testWidgets('Tapping should never cause a splash', (WidgetTester tester) async {
    final Key textField1 = UniqueKey();
    final Key textField2 = UniqueKey();

    await tester.pumpWidget(
      MaterialApp(
        home: Theme(
          data: ThemeData.light().copyWith(splashFactory: const TestInkSplashFactory()),
          child: Material(
            child: Container(
              alignment: Alignment.topLeft,
              child: Column(
                children: <Widget>[
                  TextField(key: textField1, decoration: const InputDecoration(labelText: 'label')),
                  TextField(key: textField2, decoration: const InputDecoration(labelText: 'label')),
                ],
              ),
            ),
          ),
        ),
      ),
    );

    await tester.tap(find.byKey(textField1));
    await tester.pumpAndSettle();
    expect(confirmCalled, isFalse);
    expect(cancelCalled, isFalse);

    await tester.tap(find.byKey(textField1));
    await tester.pumpAndSettle();
    expect(confirmCalled, isFalse);
    expect(cancelCalled, isFalse);

    await tester.tap(find.byKey(textField2));
    await tester.pumpAndSettle();
    expect(confirmCalled, isFalse);
    expect(cancelCalled, isFalse);

    await tester.tapAt(tester.getTopLeft(find.byKey(textField1)));
    await tester.pumpAndSettle();
    expect(confirmCalled, isFalse);
    expect(cancelCalled, isFalse);

    await tester.tap(find.byKey(textField2));
    await tester.pumpAndSettle();
    expect(confirmCalled, isFalse);
    expect(cancelCalled, isFalse);
  });

  testWidgets('Splash should never be created or canceled', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Theme(
          data: ThemeData.light().copyWith(splashFactory: const TestInkSplashFactory()),
          child: Material(
            child: ListView(
              children: <Widget>[
                const TextField(decoration: InputDecoration(labelText: 'label1')),
                const TextField(decoration: InputDecoration(labelText: 'label2')),
                Container(height: 1000.0, color: const Color(0xFF00FF00)),
              ],
            ),
          ),
        ),
      ),
    );

    // If there were a splash, this would cancel the splash.
    final TestGesture gesture1 = await tester.startGesture(tester.getCenter(find.text('label1')));

    await tester.pump(kPressTimeout);

    await gesture1.moveTo(const Offset(400.0, 300.0));
    await gesture1.up();
    expect(confirmCalled, isFalse);
    expect(cancelCalled, isFalse);

    // Pointer is dragged upwards causing a scroll, splash would be canceled.
    final TestGesture gesture2 = await tester.startGesture(tester.getCenter(find.text('label2')));
    await tester.pump(kPressTimeout);
    await gesture2.moveBy(const Offset(0.0, -200.0));
    await gesture2.up();
    expect(confirmCalled, isFalse);
    expect(cancelCalled, isFalse);
  });
}
