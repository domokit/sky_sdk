// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

@Tags(<String>['reduced-test-set'])
library;

import 'package:flutter/cupertino.dart';
import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  final MagnifierController magnifierController = MagnifierController();
  const Rect reasonableTextField = Rect.fromLTRB(50, 100, 200, 100);
  final Offset basicOffset = Offset(
    Magnifier.kDefaultMagnifierSize.width / 2,
    Magnifier.kStandardVerticalFocalPointShift + Magnifier.kDefaultMagnifierSize.height,
  );

  Offset getMagnifierPosition(WidgetTester tester, [bool animated = false]) {
    if (animated) {
      final AnimatedPositioned animatedPositioned = tester.firstWidget(
        find.byType(AnimatedPositioned),
      );
      return Offset(animatedPositioned.left ?? 0, animatedPositioned.top ?? 0);
    } else {
      final Positioned positioned = tester.firstWidget(find.byType(Positioned));
      return Offset(positioned.left ?? 0, positioned.top ?? 0);
    }
  }

  Future<void> showMagnifier(
    BuildContext context,
    WidgetTester tester,
    ValueNotifier<MagnifierInfo> magnifierInfo,
  ) async {
    final Future<void> magnifierShown = magnifierController.show(
      context: context,
      builder: (_) => TextMagnifier(magnifierInfo: magnifierInfo),
    );

    WidgetsBinding.instance.scheduleFrame();
    await tester.pumpAndSettle();

    // Verify that the magnifier is shown.
    await magnifierShown;
  }

  tearDown(() {
    magnifierController.removeFromOverlay();
    magnifierController.animationController = null;
  });

  group('adaptiveMagnifierControllerBuilder', () {
    testWidgets(
      'should return a TextEditingMagnifier on Android',
      (WidgetTester tester) async {
        await tester.pumpWidget(const MaterialApp(home: Placeholder()));

        final BuildContext context = tester.firstElement(find.byType(Placeholder));

        final ValueNotifier<MagnifierInfo> magnifierPositioner = ValueNotifier<MagnifierInfo>(
          MagnifierInfo.empty,
        );
        addTearDown(magnifierPositioner.dispose);

        final Widget? builtWidget = TextMagnifier.adaptiveMagnifierConfiguration.magnifierBuilder(
          context,
          MagnifierController(),
          magnifierPositioner,
        );

        expect(builtWidget, isA<TextMagnifier>());
      },
      variant: TargetPlatformVariant.only(TargetPlatform.android),
    );

    testWidgets(
      'should return a CupertinoMagnifier on iOS',
      (WidgetTester tester) async {
        await tester.pumpWidget(const MaterialApp(home: Placeholder()));

        final BuildContext context = tester.firstElement(find.byType(Placeholder));

        final ValueNotifier<MagnifierInfo> magnifierPositioner = ValueNotifier<MagnifierInfo>(
          MagnifierInfo.empty,
        );
        addTearDown(magnifierPositioner.dispose);

        final Widget? builtWidget = TextMagnifier.adaptiveMagnifierConfiguration.magnifierBuilder(
          context,
          MagnifierController(),
          magnifierPositioner,
        );

        expect(builtWidget, isA<CupertinoTextMagnifier>());
      },
      variant: TargetPlatformVariant.only(TargetPlatform.iOS),
    );

    testWidgets(
      'should return null on all platforms not Android, iOS',
      (WidgetTester tester) async {
        await tester.pumpWidget(const MaterialApp(home: Placeholder()));

        final BuildContext context = tester.firstElement(find.byType(Placeholder));

        final ValueNotifier<MagnifierInfo> magnifierPositioner = ValueNotifier<MagnifierInfo>(
          MagnifierInfo.empty,
        );
        addTearDown(magnifierPositioner.dispose);

        final Widget? builtWidget = TextMagnifier.adaptiveMagnifierConfiguration.magnifierBuilder(
          context,
          MagnifierController(),
          magnifierPositioner,
        );

        expect(builtWidget, isNull);
      },
      variant: TargetPlatformVariant.all(
        excluding: <TargetPlatform>{TargetPlatform.iOS, TargetPlatform.android},
      ),
    );
  });

  group('magnifier', () {
    group('position', () {
      testWidgets('should be at gesture position if does not violate any positioning rules', (
        WidgetTester tester,
      ) async {
        final Key textField = UniqueKey();

        await tester.pumpWidget(const MaterialApp(home: Placeholder()));

        await tester.pumpWidget(
          ColoredBox(
            color: const Color.fromARGB(255, 0, 255, 179),
            child: MaterialApp(
              home: Center(
                child: Container(
                  key: textField,
                  width: 10,
                  height: 10,
                  color: Colors.red,
                  child: const Placeholder(),
                ),
              ),
            ),
          ),
        );

        final BuildContext context = tester.firstElement(find.byType(Placeholder));

        // Magnifier should be positioned directly over the red square.
        final RenderBox tapPointRenderBox =
            tester.firstRenderObject(find.byKey(textField)) as RenderBox;
        final Rect fakeTextFieldRect =
            tapPointRenderBox.localToGlobal(Offset.zero) & tapPointRenderBox.size;

        final ValueNotifier<MagnifierInfo> magnifierInfo = ValueNotifier<MagnifierInfo>(
          MagnifierInfo(
            currentLineBoundaries: fakeTextFieldRect,
            fieldBounds: fakeTextFieldRect,
            caretRect: fakeTextFieldRect,
            // The tap position is dragBelow units below the text field.
            globalGesturePosition: fakeTextFieldRect.center,
          ),
        );
        addTearDown(magnifierInfo.dispose);

        await showMagnifier(context, tester, magnifierInfo);

        // Should show two red crossed-out squares: the original in the center,
        // and one in the magnifier, in the upper half of the image, surrounded
        // by a faint offset rounded rectangle shadow.
        await expectLater(
          find.byType(MaterialApp),
          matchesGoldenFile('magnifier.position.default.png'),
        );
      });

      testWidgets('should never move outside the right bounds of the editing line', (
        WidgetTester tester,
      ) async {
        const double gestureOutsideLine = 100;

        await tester.pumpWidget(const MaterialApp(home: Placeholder()));

        final BuildContext context = tester.firstElement(find.byType(Placeholder));

        late ValueNotifier<MagnifierInfo> magnifierPositioner;
        addTearDown(() => magnifierPositioner.dispose());

        await showMagnifier(
          context,
          tester,
          magnifierPositioner = ValueNotifier<MagnifierInfo>(
            MagnifierInfo(
              currentLineBoundaries: reasonableTextField,
              // Inflate these two to make sure we're bounding on the
              // current line boundaries, not anything else.
              fieldBounds: reasonableTextField.inflate(gestureOutsideLine),
              caretRect: reasonableTextField.inflate(gestureOutsideLine),
              // The tap position is far out of the right side of the app.
              globalGesturePosition: Offset(reasonableTextField.right + gestureOutsideLine, 0),
            ),
          ),
        );

        // Should be less than the right edge, since we have padding.
        expect(getMagnifierPosition(tester).dx, lessThanOrEqualTo(reasonableTextField.right));
      });

      testWidgets('should never move outside the left bounds of the editing line', (
        WidgetTester tester,
      ) async {
        const double gestureOutsideLine = 100;

        await tester.pumpWidget(const MaterialApp(home: Placeholder()));

        final BuildContext context = tester.firstElement(find.byType(Placeholder));

        late ValueNotifier<MagnifierInfo> magnifierPositioner;
        addTearDown(() => magnifierPositioner.dispose());

        await showMagnifier(
          context,
          tester,
          magnifierPositioner = ValueNotifier<MagnifierInfo>(
            MagnifierInfo(
              currentLineBoundaries: reasonableTextField,
              // Inflate these two to make sure we're bounding on the
              // current line boundaries, not anything else.
              fieldBounds: reasonableTextField.inflate(gestureOutsideLine),
              caretRect: reasonableTextField.inflate(gestureOutsideLine),
              // The tap position is far out of the left side of the app.
              globalGesturePosition: Offset(reasonableTextField.left - gestureOutsideLine, 0),
            ),
          ),
        );

        expect(
          getMagnifierPosition(tester).dx + basicOffset.dx,
          greaterThanOrEqualTo(reasonableTextField.left),
        );
      });

      testWidgets('should position vertically at the center of the line', (
        WidgetTester tester,
      ) async {
        await tester.pumpWidget(const MaterialApp(home: Placeholder()));

        final BuildContext context = tester.firstElement(find.byType(Placeholder));

        late ValueNotifier<MagnifierInfo> magnifierPositioner;
        addTearDown(() => magnifierPositioner.dispose());

        await showMagnifier(
          context,
          tester,
          magnifierPositioner = ValueNotifier<MagnifierInfo>(
            MagnifierInfo(
              currentLineBoundaries: reasonableTextField,
              fieldBounds: reasonableTextField,
              caretRect: reasonableTextField,
              globalGesturePosition: reasonableTextField.center,
            ),
          ),
        );

        expect(getMagnifierPosition(tester).dy, reasonableTextField.center.dy - basicOffset.dy);
      });

      testWidgets('should reposition vertically if mashed against the ceiling', (
        WidgetTester tester,
      ) async {
        final Rect topOfScreenTextFieldRect = Rect.fromPoints(Offset.zero, const Offset(200, 0));

        await tester.pumpWidget(const MaterialApp(home: Placeholder()));

        final BuildContext context = tester.firstElement(find.byType(Placeholder));

        late ValueNotifier<MagnifierInfo> magnifierPositioner;
        addTearDown(() => magnifierPositioner.dispose());

        await showMagnifier(
          context,
          tester,
          magnifierPositioner = ValueNotifier<MagnifierInfo>(
            MagnifierInfo(
              currentLineBoundaries: topOfScreenTextFieldRect,
              fieldBounds: topOfScreenTextFieldRect,
              caretRect: topOfScreenTextFieldRect,
              globalGesturePosition: topOfScreenTextFieldRect.topCenter,
            ),
          ),
        );

        expect(getMagnifierPosition(tester).dy, greaterThanOrEqualTo(0));
      });
    });

    group('focal point', () {
      Offset getMagnifierAdditionalFocalPoint(WidgetTester tester) {
        final Magnifier magnifier = tester.firstWidget(find.byType(Magnifier));
        return magnifier.additionalFocalPointOffset;
      }

      testWidgets('should shift focal point so that the lens sees nothing out of bounds', (
        WidgetTester tester,
      ) async {
        await tester.pumpWidget(const MaterialApp(home: Placeholder()));

        final BuildContext context = tester.firstElement(find.byType(Placeholder));

        late ValueNotifier<MagnifierInfo> magnifierPositioner;
        addTearDown(() => magnifierPositioner.dispose());

        await showMagnifier(
          context,
          tester,
          magnifierPositioner = ValueNotifier<MagnifierInfo>(
            MagnifierInfo(
              currentLineBoundaries: reasonableTextField,
              fieldBounds: reasonableTextField,
              caretRect: reasonableTextField,
              // Gesture on the far right of the magnifier.
              globalGesturePosition: reasonableTextField.topLeft,
            ),
          ),
        );

        expect(getMagnifierAdditionalFocalPoint(tester).dx, lessThan(reasonableTextField.left));
      });

      testWidgets('focal point should shift if mashed against the top to always point to text', (
        WidgetTester tester,
      ) async {
        final Rect topOfScreenTextFieldRect = Rect.fromPoints(Offset.zero, const Offset(200, 0));

        await tester.pumpWidget(const MaterialApp(home: Placeholder()));

        final BuildContext context = tester.firstElement(find.byType(Placeholder));

        late ValueNotifier<MagnifierInfo> magnifierPositioner;
        addTearDown(() => magnifierPositioner.dispose());

        await showMagnifier(
          context,
          tester,
          magnifierPositioner = ValueNotifier<MagnifierInfo>(
            MagnifierInfo(
              currentLineBoundaries: topOfScreenTextFieldRect,
              fieldBounds: topOfScreenTextFieldRect,
              caretRect: topOfScreenTextFieldRect,
              globalGesturePosition: topOfScreenTextFieldRect.topCenter,
            ),
          ),
        );

        expect(getMagnifierAdditionalFocalPoint(tester).dy, lessThan(0));
      });
    });

    group('animation state', () {
      bool getIsAnimated(WidgetTester tester) {
        final AnimatedPositioned animatedPositioned = tester.firstWidget(
          find.byType(AnimatedPositioned),
        );
        return animatedPositioned.duration.compareTo(Duration.zero) != 0;
      }

      testWidgets('should not be animated on the initial state', (WidgetTester tester) async {
        await tester.pumpWidget(const MaterialApp(home: Placeholder()));

        final BuildContext context = tester.firstElement(find.byType(Placeholder));

        late ValueNotifier<MagnifierInfo> magnifierInfo;
        addTearDown(() => magnifierInfo.dispose());

        await showMagnifier(
          context,
          tester,
          magnifierInfo = ValueNotifier<MagnifierInfo>(
            MagnifierInfo(
              currentLineBoundaries: reasonableTextField,
              fieldBounds: reasonableTextField,
              caretRect: reasonableTextField,
              globalGesturePosition: reasonableTextField.center,
            ),
          ),
        );

        expect(getIsAnimated(tester), false);
      });

      testWidgets('should not be animated on horizontal shifts', (WidgetTester tester) async {
        await tester.pumpWidget(const MaterialApp(home: Placeholder()));

        final BuildContext context = tester.firstElement(find.byType(Placeholder));

        final ValueNotifier<MagnifierInfo> magnifierPositioner = ValueNotifier<MagnifierInfo>(
          MagnifierInfo(
            currentLineBoundaries: reasonableTextField,
            fieldBounds: reasonableTextField,
            caretRect: reasonableTextField,
            globalGesturePosition: reasonableTextField.center,
          ),
        );
        addTearDown(magnifierPositioner.dispose);

        await showMagnifier(context, tester, magnifierPositioner);

        // New position has a horizontal shift.
        magnifierPositioner.value = MagnifierInfo(
          currentLineBoundaries: reasonableTextField,
          fieldBounds: reasonableTextField,
          caretRect: reasonableTextField,
          globalGesturePosition: reasonableTextField.center + const Offset(200, 0),
        );
        await tester.pumpAndSettle();

        expect(getIsAnimated(tester), false);
      });

      testWidgets('should be animated on vertical shifts', (WidgetTester tester) async {
        const Offset verticalShift = Offset(0, 200);

        await tester.pumpWidget(const MaterialApp(home: Placeholder()));

        final BuildContext context = tester.firstElement(find.byType(Placeholder));

        final ValueNotifier<MagnifierInfo> magnifierPositioner = ValueNotifier<MagnifierInfo>(
          MagnifierInfo(
            currentLineBoundaries: reasonableTextField,
            fieldBounds: reasonableTextField,
            caretRect: reasonableTextField,
            globalGesturePosition: reasonableTextField.center,
          ),
        );
        addTearDown(magnifierPositioner.dispose);

        await showMagnifier(context, tester, magnifierPositioner);

        // New position has a vertical shift.
        magnifierPositioner.value = MagnifierInfo(
          currentLineBoundaries: reasonableTextField.shift(verticalShift),
          fieldBounds: Rect.fromPoints(
            reasonableTextField.topLeft,
            reasonableTextField.bottomRight + verticalShift,
          ),
          caretRect: reasonableTextField.shift(verticalShift),
          globalGesturePosition: reasonableTextField.center + verticalShift,
        );

        await tester.pump();
        expect(getIsAnimated(tester), true);
      });

      testWidgets('should stop being animated when timer is up', (WidgetTester tester) async {
        const Offset verticalShift = Offset(0, 200);

        await tester.pumpWidget(const MaterialApp(home: Placeholder()));

        final BuildContext context = tester.firstElement(find.byType(Placeholder));

        final ValueNotifier<MagnifierInfo> magnifierPositioner = ValueNotifier<MagnifierInfo>(
          MagnifierInfo(
            currentLineBoundaries: reasonableTextField,
            fieldBounds: reasonableTextField,
            caretRect: reasonableTextField,
            globalGesturePosition: reasonableTextField.center,
          ),
        );
        addTearDown(magnifierPositioner.dispose);

        await showMagnifier(context, tester, magnifierPositioner);

        // New position has a vertical shift.
        magnifierPositioner.value = MagnifierInfo(
          currentLineBoundaries: reasonableTextField.shift(verticalShift),
          fieldBounds: Rect.fromPoints(
            reasonableTextField.topLeft,
            reasonableTextField.bottomRight + verticalShift,
          ),
          caretRect: reasonableTextField.shift(verticalShift),
          globalGesturePosition: reasonableTextField.center + verticalShift,
        );

        await tester.pump();
        expect(getIsAnimated(tester), true);
        await tester.pump(
          TextMagnifier.jumpBetweenLinesAnimationDuration + const Duration(seconds: 2),
        );
        expect(getIsAnimated(tester), false);
      });
    });
  });
}
