// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This file is run as part of a reduced test set in CI on Mac and Windows
// machines.
@Tags(<String>['reduced-test-set'])
library;

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('The Ink widget expands when no dimensions are set', (WidgetTester tester) async {
    await tester.pumpWidget(Material(child: Ink()));
    expect(find.byType(Ink), findsOneWidget);
    expect(tester.getSize(find.byType(Ink)), const Size(800.0, 600.0));
  });

  testWidgets('The Ink widget fits the specified size', (WidgetTester tester) async {
    const double height = 150.0;
    const double width = 200.0;
    await tester.pumpWidget(
      Material(
        child: Center(
          // used to constrain to child's size
          child: Ink(height: height, width: width),
        ),
      ),
    );
    await tester.pumpAndSettle();
    expect(find.byType(Ink), findsOneWidget);
    expect(tester.getSize(find.byType(Ink)), const Size(width, height));
  });

  testWidgets('The Ink widget expands on a unspecified dimension', (WidgetTester tester) async {
    const double height = 150.0;
    await tester.pumpWidget(
      Material(
        child: Center(
          // used to constrain to child's size
          child: Ink(height: height),
        ),
      ),
    );
    await tester.pumpAndSettle();
    expect(find.byType(Ink), findsOneWidget);
    expect(tester.getSize(find.byType(Ink)), const Size(800, height));
  });

  testWidgets('Material2 - InkWell widget renders an ink splash', (WidgetTester tester) async {
    const Color splashColor = Color(0xAA0000FF);
    const BorderRadius borderRadius = BorderRadius.all(Radius.circular(6.0));

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Material(
          child: Center(
            child: SizedBox(
              width: 200.0,
              height: 60.0,
              child: InkWell(borderRadius: borderRadius, splashColor: splashColor, onTap: () {}),
            ),
          ),
        ),
      ),
    );

    final Offset center = tester.getCenter(find.byType(InkWell));
    final TestGesture gesture = await tester.startGesture(center);
    await tester.pump(); // start gesture
    await tester.pump(const Duration(milliseconds: 200)); // wait for splash to be well under way

    final RenderBox box = Material.of(tester.element(find.byType(InkWell))) as RenderBox;
    expect(
      box,
      paints
        ..translate(x: 0.0, y: 0.0)
        ..save()
        ..translate(x: 300.0, y: 270.0)
        ..clipRRect(rrect: RRect.fromLTRBR(0.0, 0.0, 200.0, 60.0, const Radius.circular(6.0)))
        ..circle(x: 100.0, y: 30.0, radius: 21.0, color: splashColor)
        ..restore(),
    );

    await gesture.up();
  });

  testWidgets('Material3 - InkWell widget renders an ink splash', (WidgetTester tester) async {
    const Key inkWellKey = Key('InkWell');
    const Color splashColor = Color(0xAA0000FF);
    const BorderRadius borderRadius = BorderRadius.all(Radius.circular(6.0));

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Center(
            child: SizedBox(
              width: 200.0,
              height: 60.0,
              child: InkWell(
                key: inkWellKey,
                borderRadius: borderRadius,
                splashColor: splashColor,
                onTap: () {},
              ),
            ),
          ),
        ),
      ),
    );

    final Offset center = tester.getCenter(find.byType(InkWell));
    final TestGesture gesture = await tester.startGesture(center);
    await tester.pump(); // start gesture
    await tester.pump(const Duration(milliseconds: 200)); // wait for splash to be well under way

    final RenderBox box = Material.of(tester.element(find.byType(InkWell))) as RenderBox;
    if (kIsWeb) {
      expect(
        box,
        paints
          ..save()
          ..translate(x: 0.0, y: 0.0)
          ..clipRect()
          ..save()
          ..translate(x: 300.0, y: 270.0)
          ..clipRRect(rrect: RRect.fromLTRBR(0.0, 0.0, 200.0, 60.0, const Radius.circular(6.0)))
          ..circle()
          ..restore(),
      );
    } else {
      expect(
        box,
        paints
          ..translate(x: 0.0, y: 0.0)
          ..save()
          ..translate(x: 300.0, y: 270.0)
          ..clipRRect(rrect: RRect.fromLTRBR(0.0, 0.0, 200.0, 60.0, const Radius.circular(6.0)))
          ..rect(rect: const Rect.fromLTRB(0.0, 0.0, 200, 60))
          ..restore(),
      );
    }

    // Material 3 uses the InkSparkle which uses a shader, so we can't capture
    // the effect with paint methods. Use a golden test instead.
    await expectLater(
      find.byKey(inkWellKey),
      matchesGoldenFile('m3_ink_well.renders.ink_splash.png'),
    );

    await gesture.up();
  });

  testWidgets('The InkWell widget renders an ink ripple', (WidgetTester tester) async {
    const Color highlightColor = Color(0xAAFF0000);
    const Color splashColor = Color(0xB40000FF);
    const BorderRadius borderRadius = BorderRadius.all(Radius.circular(6.0));

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Material(
          child: Center(
            child: SizedBox(
              width: 100.0,
              height: 100.0,
              child: InkWell(
                borderRadius: borderRadius,
                highlightColor: highlightColor,
                splashColor: splashColor,
                onTap: () {},
                radius: 100.0,
                splashFactory: InkRipple.splashFactory,
              ),
            ),
          ),
        ),
      ),
    );

    final Offset tapDownOffset = tester.getTopLeft(find.byType(InkWell));
    final Offset inkWellCenter = tester.getCenter(find.byType(InkWell));
    //final TestGesture gesture = await tester.startGesture(tapDownOffset);
    await tester.tapAt(tapDownOffset);
    await tester.pump(); // start gesture

    final RenderBox box = Material.of(tester.element(find.byType(InkWell))) as RenderBox;

    bool offsetsAreClose(Offset a, Offset b) => (a - b).distance < 1.0;
    bool radiiAreClose(double a, double b) => (a - b).abs() < 1.0;

    PaintPattern ripplePattern(Offset expectedCenter, double expectedRadius, int expectedAlpha) {
      return paints
        ..translate(x: 0.0, y: 0.0)
        ..translate(x: tapDownOffset.dx, y: tapDownOffset.dy)
        ..something((Symbol method, List<dynamic> arguments) {
          if (method != #drawCircle) {
            return false;
          }
          final Offset center = arguments[0] as Offset;
          final double radius = arguments[1] as double;
          final Paint paint = arguments[2] as Paint;
          if (offsetsAreClose(center, expectedCenter) &&
              radiiAreClose(radius, expectedRadius) &&
              paint.color.alpha == expectedAlpha) {
            return true;
          }
          throw '''
            Expected: center == $expectedCenter, radius == $expectedRadius, alpha == $expectedAlpha
            Found: center == $center radius == $radius alpha == ${paint.color.alpha}''';
        });
    }

    // Initially the ripple's center is where the tap occurred;
    // ripplePattern always add a translation of tapDownOffset.
    expect(box, ripplePattern(Offset.zero, 30.0, 0));

    // The ripple fades in for 75ms. During that time its alpha is eased from
    // 0 to the splashColor's alpha value and its center moves towards the
    // center of the ink well.
    await tester.pump(const Duration(milliseconds: 50));
    expect(box, ripplePattern(const Offset(17.0, 17.0), 56.0, 120));

    // At 75ms the ripple has fade in: it's alpha matches the splashColor's
    // alpha and its center has moved closer to the ink well's center.
    await tester.pump(const Duration(milliseconds: 25));
    expect(box, ripplePattern(const Offset(29.0, 29.0), 73.0, 180));

    // At this point the splash radius has expanded to its limit: 5 past the
    // ink well's radius parameter. The splash center has moved to its final
    // location at the inkwell's center and the fade-out is about to start.
    // The fade-out begins at 225ms = 50ms + 25ms + 150ms.
    await tester.pump(const Duration(milliseconds: 150));
    expect(box, ripplePattern(inkWellCenter - tapDownOffset, 105.0, 180));

    // After another 150ms the fade-out is complete.
    await tester.pump(const Duration(milliseconds: 150));
    expect(box, ripplePattern(inkWellCenter - tapDownOffset, 105.0, 0));
  });

  testWidgets('Material2 - Does the Ink widget render anything', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Material(
          child: Center(
            child: Ink(
              color: Colors.blue,
              width: 200.0,
              height: 200.0,
              child: InkWell(splashColor: Colors.green, onTap: () {}),
            ),
          ),
        ),
      ),
    );

    final Offset center = tester.getCenter(find.byType(InkWell));
    final TestGesture gesture = await tester.startGesture(center);
    await tester.pump(); // start gesture
    await tester.pump(const Duration(milliseconds: 200)); // wait for splash to be well under way

    final RenderBox box = Material.of(tester.element(find.byType(InkWell))) as RenderBox;
    expect(
      box,
      paints
        ..rect(
          rect: const Rect.fromLTRB(300.0, 200.0, 500.0, 400.0),
          color: Color(Colors.blue.value),
        )
        ..circle(color: Color(Colors.green.value)),
    );

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Material(
          child: Center(
            child: Ink(
              color: Colors.red,
              width: 200.0,
              height: 200.0,
              child: InkWell(splashColor: Colors.green, onTap: () {}),
            ),
          ),
        ),
      ),
    );

    expect(Material.of(tester.element(find.byType(InkWell))), same(box));

    expect(
      box,
      paints
        ..rect(
          rect: const Rect.fromLTRB(300.0, 200.0, 500.0, 400.0),
          color: Color(Colors.red.value),
        )
        ..circle(color: Color(Colors.green.value)),
    );

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Material(
          child: Center(
            child: InkWell(
              // this is at a different depth in the tree so it's now a new InkWell
              splashColor: Colors.green,
              onTap: () {},
            ),
          ),
        ),
      ),
    );

    expect(Material.of(tester.element(find.byType(InkWell))), same(box));

    expect(box, isNot(paints..rect()));
    expect(box, isNot(paints..circle()));

    await gesture.up();
  });

  testWidgets('Material3 - Does the Ink widget render anything', (WidgetTester tester) async {
    const Key inkWellKey = Key('InkWell');
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Center(
            child: Ink(
              color: Colors.blue,
              width: 200.0,
              height: 200.0,
              child: InkWell(key: inkWellKey, splashColor: Colors.green, onTap: () {}),
            ),
          ),
        ),
      ),
    );

    final Offset center = tester.getCenter(find.byType(InkWell));
    final TestGesture gesture = await tester.startGesture(center);
    await tester.pump(); // start gesture
    await tester.pump(const Duration(milliseconds: 200)); // wait for splash to be well under way

    final RenderBox box = Material.of(tester.element(find.byType(InkWell))) as RenderBox;
    expect(
      box,
      paints..rect(
        rect: const Rect.fromLTRB(300.0, 200.0, 500.0, 400.0),
        color: Color(Colors.blue.value),
      ),
    );

    // Material 3 uses the InkSparkle which uses a shader, so we can't capture
    // the effect with paint methods. Use a golden test instead.
    await expectLater(find.byKey(inkWellKey), matchesGoldenFile('m3_ink.renders.anything.0.png'));

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Center(
            child: Ink(
              color: Colors.red,
              width: 200.0,
              height: 200.0,
              child: InkWell(key: inkWellKey, splashColor: Colors.green, onTap: () {}),
            ),
          ),
        ),
      ),
    );

    expect(Material.of(tester.element(find.byType(InkWell))), same(box));

    expect(
      box,
      paints..rect(
        rect: const Rect.fromLTRB(300.0, 200.0, 500.0, 400.0),
        color: Color(Colors.red.value),
      ),
    );

    // Material 3 uses the InkSparkle which uses a shader, so we can't capture
    // the effect with paint methods. Use a golden test instead.
    await expectLater(find.byKey(inkWellKey), matchesGoldenFile('m3_ink.renders.anything.1.png'));

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Center(
            child: InkWell(
              // This is at a different depth in the tree so it's now a new InkWell.
              key: inkWellKey,
              splashColor: Colors.green,
              onTap: () {},
            ),
          ),
        ),
      ),
    );

    expect(Material.of(tester.element(find.byType(InkWell))), same(box));

    expect(box, isNot(paints..rect()));
    expect(box, isNot(paints..rect()));

    await gesture.up();
  });

  testWidgets('The InkWell widget renders an SelectAction or ActivateAction-induced ink ripple', (
    WidgetTester tester,
  ) async {
    const Color highlightColor = Color(0xAAFF0000);
    const Color splashColor = Color(0xB40000FF);
    const BorderRadius borderRadius = BorderRadius.all(Radius.circular(6.0));

    final FocusNode focusNode = FocusNode(debugLabel: 'Test Node');
    addTearDown(focusNode.dispose);
    Future<void> buildTest(Intent intent) async {
      return tester.pumpWidget(
        Shortcuts(
          shortcuts: <ShortcutActivator, Intent>{
            const SingleActivator(LogicalKeyboardKey.space): intent,
          },
          child: Directionality(
            textDirection: TextDirection.ltr,
            child: Material(
              child: Center(
                child: SizedBox(
                  width: 100.0,
                  height: 100.0,
                  child: InkWell(
                    borderRadius: borderRadius,
                    highlightColor: highlightColor,
                    splashColor: splashColor,
                    focusNode: focusNode,
                    onTap: () {},
                    radius: 100.0,
                    splashFactory: InkRipple.splashFactory,
                  ),
                ),
              ),
            ),
          ),
        ),
      );
    }

    await buildTest(const ActivateIntent());
    focusNode.requestFocus();
    await tester.pumpAndSettle();

    final Offset topLeft = tester.getTopLeft(find.byType(InkWell));
    final Offset inkWellCenter = tester.getCenter(find.byType(InkWell)) - topLeft;

    bool offsetsAreClose(Offset a, Offset b) => (a - b).distance < 1.0;
    bool radiiAreClose(double a, double b) => (a - b).abs() < 1.0;

    PaintPattern ripplePattern(double expectedRadius, int expectedAlpha) {
      return paints
        ..translate(x: 0.0, y: 0.0)
        ..translate(x: topLeft.dx, y: topLeft.dy)
        ..something((Symbol method, List<dynamic> arguments) {
          if (method != #drawCircle) {
            return false;
          }
          final Offset center = arguments[0] as Offset;
          final double radius = arguments[1] as double;
          final Paint paint = arguments[2] as Paint;
          if (offsetsAreClose(center, inkWellCenter) &&
              radiiAreClose(radius, expectedRadius) &&
              paint.color.alpha == expectedAlpha) {
            return true;
          }
          throw '''
            Expected: center == $inkWellCenter, radius == $expectedRadius, alpha == $expectedAlpha
            Found: center == $center radius == $radius alpha == ${paint.color.alpha}''';
        });
    }

    await buildTest(const ActivateIntent());
    await tester.pumpAndSettle();
    await tester.sendKeyEvent(LogicalKeyboardKey.space);
    await tester.pump();

    final RenderBox box = Material.of(tester.element(find.byType(InkWell))) as RenderBox;

    // ripplePattern always add a translation of topLeft.
    expect(box, ripplePattern(30.0, 0));

    // The ripple fades in for 75ms. During that time its alpha is eased from
    // 0 to the splashColor's alpha value.
    await tester.pump(const Duration(milliseconds: 50));
    expect(box, ripplePattern(56.0, 120));

    // At 75ms the ripple has faded in: it's alpha matches the splashColor's
    // alpha.
    await tester.pump(const Duration(milliseconds: 25));
    expect(box, ripplePattern(73.0, 180));

    // At this point the splash radius has expanded to its limit: 5 past the
    // ink well's radius parameter. The fade-out is about to start.
    // The fade-out begins at 225ms = 50ms + 25ms + 150ms.
    await tester.pump(const Duration(milliseconds: 150));
    expect(box, ripplePattern(105.0, 180));

    // After another 150ms the fade-out is complete.
    await tester.pump(const Duration(milliseconds: 150));
    expect(box, ripplePattern(105.0, 0));
  });

  testWidgets('Cancel an InkRipple that was disposed when its animation ended', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/14391
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Material(
          child: Center(
            child: SizedBox(
              width: 100.0,
              height: 100.0,
              child: InkWell(onTap: () {}, radius: 100.0, splashFactory: InkRipple.splashFactory),
            ),
          ),
        ),
      ),
    );

    final Offset tapDownOffset = tester.getTopLeft(find.byType(InkWell));
    await tester.tapAt(tapDownOffset);
    await tester.pump(); // start splash
    await tester.pump(const Duration(milliseconds: 375)); // _kFadeOutDuration, in_ripple.dart

    final TestGesture gesture = await tester.startGesture(tapDownOffset);
    await tester.pump(); // start gesture
    await gesture.moveTo(Offset.zero);
    await gesture.up(); // generates a tap cancel
    await tester.pumpAndSettle();
  });

  testWidgets('Cancel an InkRipple that was disposed when its animation ended', (
    WidgetTester tester,
  ) async {
    const Color highlightColor = Color(0xAAFF0000);
    const Color splashColor = Color(0xB40000FF);

    // Regression test for https://github.com/flutter/flutter/issues/14391
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Material(
          child: Center(
            child: SizedBox(
              width: 100.0,
              height: 100.0,
              child: InkWell(
                splashColor: splashColor,
                highlightColor: highlightColor,
                onTap: () {},
                radius: 100.0,
                splashFactory: InkRipple.splashFactory,
              ),
            ),
          ),
        ),
      ),
    );

    final Offset tapDownOffset = tester.getTopLeft(find.byType(InkWell));
    await tester.tapAt(tapDownOffset);
    await tester.pump(); // start splash
    // No delay here so _fadeInController.value=1.0 (InkRipple.dart)

    // Generate a tap cancel; Will cancel the ink splash before it started
    final TestGesture gesture = await tester.startGesture(tapDownOffset);
    await tester.pump(); // start gesture
    await gesture.moveTo(Offset.zero);
    await gesture.up(); // generates a tap cancel

    final RenderBox box = Material.of(tester.element(find.byType(InkWell))) as RenderBox;
    expect(
      box,
      paints..everything((Symbol method, List<dynamic> arguments) {
        if (method != #drawCircle) {
          return true;
        }
        final Paint paint = arguments[2] as Paint;
        if (paint.color.alpha == 0) {
          return true;
        }
        throw 'Expected: paint.color.alpha == 0, found: ${paint.color.alpha}';
      }),
    );
  });

  testWidgets('The InkWell widget on OverlayPortal does not throw', (WidgetTester tester) async {
    final OverlayPortalController controller = OverlayPortalController();
    controller.show();

    late OverlayEntry overlayEntry;
    addTearDown(
      () =>
          overlayEntry
            ..remove()
            ..dispose(),
    );

    await tester.pumpWidget(
      Center(
        child: RepaintBoundary(
          child: SizedBox.square(
            dimension: 200,
            child: Directionality(
              textDirection: TextDirection.ltr,
              child: Overlay(
                initialEntries: <OverlayEntry>[
                  overlayEntry = OverlayEntry(
                    builder: (BuildContext context) {
                      return Center(
                        child: SizedBox.square(
                          dimension: 100,
                          // The material partially overlaps the overlayChild.
                          // This is to verify that the `overlayChild`'s ink
                          // features aren't clipped by it.
                          child: Material(
                            color: Colors.black,
                            child: OverlayPortal(
                              controller: controller,
                              overlayChildBuilder: (BuildContext context) {
                                return Positioned(
                                  right: 0,
                                  bottom: 0,
                                  child: InkWell(
                                    splashColor: Colors.red,
                                    onTap: () {},
                                    child: const SizedBox.square(dimension: 100),
                                  ),
                                );
                              },
                            ),
                          ),
                        ),
                      );
                    },
                  ),
                ],
              ),
            ),
          ),
        ),
      ),
    );

    final TestGesture gesture = await tester.startGesture(tester.getCenter(find.byType(InkWell)));
    addTearDown(() async {
      await gesture.up();
    });

    await tester.pump(); // start gesture
    await tester.pump(const Duration(seconds: 2));

    expect(tester.takeException(), isNull);
  });

  testWidgets('Material2 - Custom rectCallback renders an ink splash from its center', (
    WidgetTester tester,
  ) async {
    const Color splashColor = Color(0xff00ff00);

    Widget buildWidget({InteractiveInkFeatureFactory? splashFactory}) {
      return MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Material(
          child: Center(
            child: SizedBox(
              width: 100.0,
              height: 200.0,
              child: InkResponse(
                splashColor: splashColor,
                containedInkWell: true,
                highlightShape: BoxShape.rectangle,
                splashFactory: splashFactory,
                onTap: () {},
              ),
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildWidget());

    final Offset center = tester.getCenter(find.byType(SizedBox));
    TestGesture gesture = await tester.startGesture(center);
    await tester.pump(); // start gesture
    await tester.pumpAndSettle(); // Finish rendering ink splash.

    RenderBox box = Material.of(tester.element(find.byType(InkResponse))) as RenderBox;
    expect(box, paints..circle(x: 50.0, y: 100.0, color: splashColor));

    await gesture.up();

    await tester.pumpWidget(buildWidget(splashFactory: _InkRippleFactory()));
    await tester.pumpAndSettle(); // Finish rendering ink splash.

    gesture = await tester.startGesture(center);
    await tester.pump(); // start gesture
    await tester.pumpAndSettle(); // Finish rendering ink splash.

    box = Material.of(tester.element(find.byType(InkResponse))) as RenderBox;
    expect(box, paints..circle(x: 50.0, y: 50.0, color: splashColor));
  });

  testWidgets('Material3 - Custom rectCallback renders an ink splash from its center', (
    WidgetTester tester,
  ) async {
    const Key inkWResponseKey = Key('InkResponse');
    const Color splashColor = Color(0xff00ff00);

    Widget buildWidget({InteractiveInkFeatureFactory? splashFactory}) {
      return MaterialApp(
        home: Material(
          child: Center(
            child: SizedBox(
              width: 100.0,
              height: 200.0,
              child: InkResponse(
                key: inkWResponseKey,
                splashColor: splashColor,
                containedInkWell: true,
                highlightShape: BoxShape.rectangle,
                splashFactory: splashFactory,
                onTap: () {},
              ),
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildWidget());

    final Offset center = tester.getCenter(find.byType(SizedBox));
    TestGesture gesture = await tester.startGesture(center);
    await tester.pump(); // start gesture
    await tester.pumpAndSettle(); // Finish rendering ink splash.

    // Material 3 uses the InkSparkle which uses a shader, so we can't capture
    // the effect with paint methods. Use a golden test instead.
    await expectLater(
      find.byKey(inkWResponseKey),
      matchesGoldenFile('m3_ink_response.renders.ink_splash_from_its_center.0.png'),
    );

    await gesture.up();

    await tester.pumpWidget(buildWidget(splashFactory: _InkRippleFactory()));
    await tester.pumpAndSettle(); // Finish rendering ink splash.

    gesture = await tester.startGesture(center);
    await tester.pump(); // start gesture
    await tester.pumpAndSettle(); // Finish rendering ink splash.

    // Material 3 uses the InkSparkle which uses a shader, so we can't capture
    // the effect with paint methods. Use a golden test instead.
    await expectLater(
      find.byKey(inkWResponseKey),
      matchesGoldenFile('m3_ink_response.renders.ink_splash_from_its_center.1.png'),
    );
  });

  testWidgets('Ink with isVisible=false does not paint', (WidgetTester tester) async {
    const Color testColor = Color(0xffff1234);
    Widget inkWidget({required bool isVisible}) {
      return Material(
        child: Visibility.maintain(
          visible: isVisible,
          child: Ink(decoration: const BoxDecoration(color: testColor)),
        ),
      );
    }

    await tester.pumpWidget(inkWidget(isVisible: true));
    RenderBox box = tester.renderObject(find.byType(Material));
    expect(box, paints..rect(color: testColor));

    await tester.pumpWidget(inkWidget(isVisible: false));
    box = tester.renderObject(find.byType(Material));
    expect(box, isNot(paints..rect(color: testColor)));
  });
}

class _InkRippleFactory extends InteractiveInkFeatureFactory {
  @override
  InteractiveInkFeature create({
    required MaterialInkController controller,
    required RenderBox referenceBox,
    required Offset position,
    required Color color,
    required TextDirection textDirection,
    bool containedInkWell = false,
    RectCallback? rectCallback,
    BorderRadius? borderRadius,
    ShapeBorder? customBorder,
    double? radius,
    VoidCallback? onRemoved,
  }) {
    return InkRipple(
      controller: controller,
      referenceBox: referenceBox,
      position: position,
      color: color,
      containedInkWell: containedInkWell,
      rectCallback: () => Offset.zero & const Size(100, 100),
      borderRadius: borderRadius,
      customBorder: customBorder,
      radius: radius,
      onRemoved: onRemoved,
      textDirection: textDirection,
    );
  }
}
