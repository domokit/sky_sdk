// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/src/services/keyboard_key.g.dart';
import 'package:flutter_test/flutter_test.dart';
import '../widgets/feedback_tester.dart';
import '../widgets/semantics_tester.dart';

void main() {
  RenderObject getInkFeatures(WidgetTester tester) {
    return tester.allRenderObjects.firstWhere(
      (RenderObject object) => object.runtimeType.toString() == '_RenderInkFeatures',
    );
  }

  testWidgets('InkWell gestures control test', (WidgetTester tester) async {
    final List<String> log = <String>[];

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Material(
          child: Center(
            child: InkWell(
              onTap: () {
                log.add('tap');
              },
              onDoubleTap: () {
                log.add('double-tap');
              },
              onLongPress: () {
                log.add('long-press');
              },
              onTapDown: (TapDownDetails details) {
                log.add('tap-down');
              },
              onTapUp: (TapUpDetails details) {
                log.add('tap-up');
              },
              onTapCancel: () {
                log.add('tap-cancel');
              },
            ),
          ),
        ),
      ),
    );

    await tester.tap(find.byType(InkWell), pointer: 1);

    expect(log, isEmpty);

    await tester.pump(const Duration(seconds: 1));

    expect(log, equals(<String>['tap-down', 'tap-up', 'tap']));
    log.clear();

    await tester.tap(find.byType(InkWell), pointer: 2);
    await tester.pump(const Duration(milliseconds: 100));
    await tester.tap(find.byType(InkWell), pointer: 3);

    expect(log, equals(<String>['double-tap']));
    log.clear();

    await tester.longPress(find.byType(InkWell), pointer: 4);

    expect(log, equals(<String>['tap-down', 'tap-cancel', 'long-press']));

    log.clear();
    TestGesture gesture = await tester.startGesture(tester.getRect(find.byType(InkWell)).center);
    await tester.pump(const Duration(milliseconds: 100));
    expect(log, equals(<String>['tap-down']));
    await gesture.up();
    await tester.pump(const Duration(seconds: 1));
    expect(log, equals(<String>['tap-down', 'tap-up', 'tap']));

    log.clear();
    gesture = await tester.startGesture(tester.getRect(find.byType(InkWell)).center);
    await tester.pump(const Duration(milliseconds: 100));
    await gesture.moveBy(const Offset(0.0, 200.0));
    await gesture.cancel();
    expect(log, equals(<String>['tap-down', 'tap-cancel']));
  });

  testWidgets('InkWell only onTapDown enables gestures', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/96030
    bool downTapped = false;
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Material(
          child: Center(
            child: InkWell(
              onTapDown: (TapDownDetails details) {
                downTapped = true;
              },
            ),
          ),
        ),
      ),
    );

    await tester.tap(find.byType(InkWell));
    expect(downTapped, true);
  });

  testWidgets('InkWell invokes activation actions when expected', (WidgetTester tester) async {
    final List<String> log = <String>[];

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Shortcuts(
          shortcuts: const <ShortcutActivator, Intent>{
            SingleActivator(LogicalKeyboardKey.space): ActivateIntent(),
            SingleActivator(LogicalKeyboardKey.enter): ButtonActivateIntent(),
          },
          child: Material(
            child: Center(
              child: InkWell(
                autofocus: true,
                onTap: () {
                  log.add('tap');
                },
              ),
            ),
          ),
        ),
      ),
    );

    await tester.sendKeyEvent(LogicalKeyboardKey.space);
    await tester.pump();
    expect(log, equals(<String>['tap']));
    log.clear();
    await tester.sendKeyEvent(LogicalKeyboardKey.enter);
    await tester.pump();
    expect(log, equals(<String>['tap']));
  });

  testWidgets('long-press and tap on disabled should not throw', (WidgetTester tester) async {
    await tester.pumpWidget(
      const Material(
        child: Directionality(textDirection: TextDirection.ltr, child: Center(child: InkWell())),
      ),
    );
    await tester.tap(find.byType(InkWell), pointer: 1);
    await tester.pump(const Duration(seconds: 1));
    await tester.longPress(find.byType(InkWell), pointer: 1);
    await tester.pump(const Duration(seconds: 1));
  });

  testWidgets('ink well changes color on hover', (WidgetTester tester) async {
    await tester.pumpWidget(
      Material(
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: Center(
            child: SizedBox(
              width: 100,
              height: 100,
              child: InkWell(
                hoverColor: const Color(0xff00ff00),
                splashColor: const Color(0xffff0000),
                focusColor: const Color(0xff0000ff),
                highlightColor: const Color(0xf00fffff),
                onTap: () {},
                onLongPress: () {},
                onHover: (bool hover) {},
              ),
            ),
          ),
        ),
      ),
    );
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    await gesture.moveTo(tester.getCenter(find.byType(SizedBox)));
    await tester.pumpAndSettle();
    final RenderObject inkFeatures = getInkFeatures(tester);
    expect(
      inkFeatures,
      paints..rect(
        rect: const Rect.fromLTRB(350.0, 250.0, 450.0, 350.0),
        color: const Color(0xff00ff00),
      ),
    );
  });

  testWidgets('ink well changes color on hover with overlayColor', (WidgetTester tester) async {
    // Same test as 'ink well changes color on hover' except that the
    // hover color is specified with the overlayColor parameter.
    await tester.pumpWidget(
      Material(
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: Center(
            child: SizedBox(
              width: 100,
              height: 100,
              child: InkWell(
                overlayColor: MaterialStateProperty.resolveWith<Color>((Set<MaterialState> states) {
                  if (states.contains(MaterialState.hovered)) {
                    return const Color(0xff00ff00);
                  }
                  if (states.contains(MaterialState.focused)) {
                    return const Color(0xff0000ff);
                  }
                  if (states.contains(MaterialState.pressed)) {
                    return const Color(0xf00fffff);
                  }
                  return const Color(0xffbadbad); // Shouldn't happen.
                }),
                onTap: () {},
                onLongPress: () {},
                onHover: (bool hover) {},
              ),
            ),
          ),
        ),
      ),
    );
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    await gesture.moveTo(tester.getCenter(find.byType(SizedBox)));
    await tester.pumpAndSettle();
    final RenderObject inkFeatures = getInkFeatures(tester);
    expect(
      inkFeatures,
      paints..rect(
        rect: const Rect.fromLTRB(350.0, 250.0, 450.0, 350.0),
        color: const Color(0xff00ff00),
      ),
    );
  });

  testWidgets('ink response changes color on focus', (WidgetTester tester) async {
    FocusManager.instance.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    final FocusNode focusNode = FocusNode(debugLabel: 'Ink Focus');
    await tester.pumpWidget(
      Material(
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: Center(
            child: SizedBox(
              width: 100,
              height: 100,
              child: InkWell(
                focusNode: focusNode,
                hoverColor: const Color(0xff00ff00),
                splashColor: const Color(0xffff0000),
                focusColor: const Color(0xff0000ff),
                highlightColor: const Color(0xf00fffff),
                onTap: () {},
                onLongPress: () {},
                onHover: (bool hover) {},
              ),
            ),
          ),
        ),
      ),
    );
    await tester.pumpAndSettle();
    final RenderObject inkFeatures = getInkFeatures(tester);
    expect(inkFeatures, paintsExactlyCountTimes(#drawRect, 0));
    focusNode.requestFocus();
    await tester.pumpAndSettle();
    expect(
      inkFeatures,
      paints..rect(
        rect: const Rect.fromLTRB(350.0, 250.0, 450.0, 350.0),
        color: const Color(0xff0000ff),
      ),
    );
    focusNode.dispose();
  });

  testWidgets('ink response changes color on focus with overlayColor', (WidgetTester tester) async {
    // Same test as 'ink well changes color on focus' except that the
    // hover color is specified with the overlayColor parameter.
    FocusManager.instance.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    final FocusNode focusNode = FocusNode(debugLabel: 'Ink Focus');
    await tester.pumpWidget(
      Material(
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: Center(
            child: SizedBox(
              width: 100,
              height: 100,
              child: InkWell(
                focusNode: focusNode,
                overlayColor: MaterialStateProperty.resolveWith<Color>((Set<MaterialState> states) {
                  if (states.contains(MaterialState.hovered)) {
                    return const Color(0xff00ff00);
                  }
                  if (states.contains(MaterialState.focused)) {
                    return const Color(0xff0000ff);
                  }
                  if (states.contains(MaterialState.pressed)) {
                    return const Color(0xf00fffff);
                  }
                  return const Color(0xffbadbad); // Shouldn't happen.
                }),
                highlightColor: const Color(0xf00fffff),
                onTap: () {},
                onLongPress: () {},
                onHover: (bool hover) {},
              ),
            ),
          ),
        ),
      ),
    );
    await tester.pumpAndSettle();
    final RenderObject inkFeatures = getInkFeatures(tester);
    expect(inkFeatures, paintsExactlyCountTimes(#drawRect, 0));
    focusNode.requestFocus();
    await tester.pumpAndSettle();
    expect(
      inkFeatures,
      paints..rect(
        rect: const Rect.fromLTRB(350.0, 250.0, 450.0, 350.0),
        color: const Color(0xff0000ff),
      ),
    );
    focusNode.dispose();
  });

  testWidgets('ink well changes color on pressed with overlayColor', (WidgetTester tester) async {
    const Color pressedColor = Color(0xffdd00ff);

    await tester.pumpWidget(
      Material(
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: Container(
            alignment: Alignment.topLeft,
            child: SizedBox(
              width: 100,
              height: 100,
              child: InkWell(
                splashFactory: NoSplash.splashFactory,
                overlayColor: MaterialStateProperty.resolveWith<Color>((Set<MaterialState> states) {
                  if (states.contains(MaterialState.pressed)) {
                    return pressedColor;
                  }
                  return const Color(0xffbadbad); // Shouldn't happen.
                }),
                onTap: () {},
              ),
            ),
          ),
        ),
      ),
    );
    await tester.pumpAndSettle();
    final TestGesture gesture = await tester.startGesture(
      tester.getRect(find.byType(InkWell)).center,
    );
    final RenderObject inkFeatures = getInkFeatures(tester);
    expect(
      inkFeatures,
      paints..rect(rect: const Rect.fromLTRB(0, 0, 100, 100), color: pressedColor.withAlpha(0)),
    );
    await tester.pumpAndSettle(); // Let the press highlight animation finish.
    expect(
      inkFeatures,
      paints..rect(rect: const Rect.fromLTRB(0, 0, 100, 100), color: pressedColor),
    );
    await gesture.up();
  });

  group('Ink well overlayColor resolution respects WidgetState.selected', () {
    const Color selectedHoveredColor = Color(0xff00ff00);
    const Color selectedFocusedColor = Color(0xff0000ff);
    const Color selectedPressedColor = Color(0xff00ffff);
    const Rect inkRect = Rect.fromLTRB(0, 0, 100, 100);

    Widget boilerplate({FocusNode? focusNode}) {
      final WidgetStatesController statesController = WidgetStatesController(<MaterialState>{
        MaterialState.selected,
      });
      addTearDown(statesController.dispose);

      return Material(
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: Align(
            alignment: Alignment.topLeft,
            child: SizedBox(
              width: 100,
              height: 100,
              child: InkWell(
                splashFactory: NoSplash.splashFactory,
                focusNode: focusNode,
                statesController: statesController,
                overlayColor: WidgetStateProperty.resolveWith<Color>((Set<WidgetState> states) {
                  if (states.contains(WidgetState.selected)) {
                    if (states.contains(WidgetState.pressed)) {
                      return selectedPressedColor;
                    }
                    if (states.contains(WidgetState.hovered)) {
                      return selectedHoveredColor;
                    }
                    if (states.contains(WidgetState.focused)) {
                      return selectedFocusedColor;
                    }
                    return const Color(0xffbadbad); // Shouldn't happen.
                  } else {
                    return Colors.black;
                  }
                }),
                onTap: () {},
              ),
            ),
          ),
        ),
      );
    }

    testWidgets('when focused', (WidgetTester tester) async {
      FocusManager.instance.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
      final FocusNode focusNode = FocusNode(debugLabel: 'Ink Focus');
      addTearDown(focusNode.dispose);

      await tester.pumpWidget(boilerplate(focusNode: focusNode));
      await tester.pumpAndSettle();

      final RenderObject inkFeatures = getInkFeatures(tester);
      expect(inkFeatures, paintsExactlyCountTimes(#drawRect, 0));
      focusNode.requestFocus();
      await tester.pumpAndSettle();

      expect(inkFeatures, paints..rect(rect: inkRect, color: selectedFocusedColor));
    });

    testWidgets('when hovered', (WidgetTester tester) async {
      await tester.pumpWidget(boilerplate());
      await tester.pumpAndSettle();

      final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
      await gesture.addPointer();
      await gesture.moveTo(tester.getCenter(find.byType(SizedBox)));
      await tester.pumpAndSettle();

      final RenderObject inkFeatures = getInkFeatures(tester);
      expect(inkFeatures, paints..rect(rect: inkRect, color: selectedHoveredColor));
    });

    testWidgets('when pressed', (WidgetTester tester) async {
      await tester.pumpWidget(boilerplate());
      await tester.pumpAndSettle();

      final TestGesture gesture = await tester.startGesture(
        tester.getRect(find.byType(InkWell)).center,
      );
      final RenderObject inkFeatures = getInkFeatures(tester);
      expect(inkFeatures, paints..rect(rect: inkRect, color: selectedPressedColor.withAlpha(0)));
      await tester.pumpAndSettle(); // Let the press highlight animation finish.
      expect(inkFeatures, paints..rect(rect: inkRect, color: selectedPressedColor));
      await gesture.up();
    });
  });

  testWidgets('ink response splashColor matches splashColor parameter', (
    WidgetTester tester,
  ) async {
    FocusManager.instance.highlightStrategy = FocusHighlightStrategy.alwaysTouch;
    final FocusNode focusNode = FocusNode(debugLabel: 'Ink Focus');
    const Color splashColor = Color(0xffff0000);
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Material(
          child: Directionality(
            textDirection: TextDirection.ltr,
            child: Center(
              child: Focus(
                focusNode: focusNode,
                child: SizedBox(
                  width: 100,
                  height: 100,
                  child: InkWell(
                    hoverColor: const Color(0xff00ff00),
                    splashColor: splashColor,
                    focusColor: const Color(0xff0000ff),
                    highlightColor: const Color(0xf00fffff),
                    onTap: () {},
                    onLongPress: () {},
                    onHover: (bool hover) {},
                  ),
                ),
              ),
            ),
          ),
        ),
      ),
    );
    await tester.pumpAndSettle();
    final TestGesture gesture = await tester.startGesture(
      tester.getRect(find.byType(InkWell)).center,
    );
    await tester.pump(const Duration(milliseconds: 200)); // unconfirmed splash is well underway
    final RenderObject inkFeatures = getInkFeatures(tester);
    expect(inkFeatures, paints..circle(x: 50, y: 50, color: splashColor));
    await gesture.up();
    focusNode.dispose();
  });

  testWidgets('ink response splashColor matches resolved overlayColor for MaterialState.pressed', (
    WidgetTester tester,
  ) async {
    // Same test as 'ink response splashColor matches splashColor
    // parameter' except that the splash color is specified with the
    // overlayColor parameter.
    FocusManager.instance.highlightStrategy = FocusHighlightStrategy.alwaysTouch;
    final FocusNode focusNode = FocusNode(debugLabel: 'Ink Focus');
    const Color splashColor = Color(0xffff0000);
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Material(
          child: Directionality(
            textDirection: TextDirection.ltr,
            child: Center(
              child: Focus(
                focusNode: focusNode,
                child: SizedBox(
                  width: 100,
                  height: 100,
                  child: InkWell(
                    overlayColor: MaterialStateProperty.resolveWith<Color>((
                      Set<MaterialState> states,
                    ) {
                      if (states.contains(MaterialState.hovered)) {
                        return const Color(0xff00ff00);
                      }
                      if (states.contains(MaterialState.focused)) {
                        return const Color(0xff0000ff);
                      }
                      if (states.contains(MaterialState.pressed)) {
                        return splashColor;
                      }
                      return const Color(0xffbadbad); // Shouldn't happen.
                    }),
                    onTap: () {},
                    onLongPress: () {},
                    onHover: (bool hover) {},
                  ),
                ),
              ),
            ),
          ),
        ),
      ),
    );
    await tester.pumpAndSettle();
    final TestGesture gesture = await tester.startGesture(
      tester.getRect(find.byType(InkWell)).center,
    );
    await tester.pump(const Duration(milliseconds: 200)); // unconfirmed splash is well underway
    final RenderObject inkFeatures = getInkFeatures(tester);
    expect(inkFeatures, paints..circle(x: 50, y: 50, color: splashColor));
    await gesture.up();
    focusNode.dispose();
  });

  testWidgets('ink response uses radius for focus highlight', (WidgetTester tester) async {
    FocusManager.instance.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    final FocusNode focusNode = FocusNode(debugLabel: 'Ink Focus');
    await tester.pumpWidget(
      Material(
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: Center(
            child: SizedBox(
              width: 100,
              height: 100,
              child: InkResponse(
                focusNode: focusNode,
                radius: 20,
                focusColor: const Color(0xff0000ff),
                onTap: () {},
              ),
            ),
          ),
        ),
      ),
    );
    await tester.pumpAndSettle();
    final RenderObject inkFeatures = getInkFeatures(tester);
    expect(inkFeatures, paintsExactlyCountTimes(#drawCircle, 0));
    focusNode.requestFocus();
    await tester.pumpAndSettle();
    expect(inkFeatures, paints..circle(radius: 20, color: const Color(0xff0000ff)));
    focusNode.dispose();
  });

  testWidgets('InkWell uses borderRadius for focus highlight', (WidgetTester tester) async {
    FocusManager.instance.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    final FocusNode focusNode = FocusNode(debugLabel: 'Ink Focus');
    await tester.pumpWidget(
      Material(
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: Center(
            child: SizedBox(
              width: 100,
              height: 100,
              child: InkWell(
                focusNode: focusNode,
                borderRadius: BorderRadius.circular(10),
                focusColor: const Color(0xff0000ff),
                onTap: () {},
              ),
            ),
          ),
        ),
      ),
    );
    await tester.pumpAndSettle();
    final RenderObject inkFeatures = getInkFeatures(tester);
    expect(inkFeatures, paintsExactlyCountTimes(#drawRRect, 0));

    focusNode.requestFocus();
    await tester.pumpAndSettle();
    expect(inkFeatures, paintsExactlyCountTimes(#drawRRect, 1));

    expect(
      inkFeatures,
      paints..rrect(
        rrect: RRect.fromLTRBR(350.0, 250.0, 450.0, 350.0, const Radius.circular(10)),
        color: const Color(0xff0000ff),
      ),
    );
    focusNode.dispose();
  });

  testWidgets('InkWell uses borderRadius for hover highlight', (WidgetTester tester) async {
    await tester.pumpWidget(
      Material(
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: Center(
            child: SizedBox(
              width: 100,
              height: 100,
              child: MouseRegion(
                child: InkWell(
                  borderRadius: BorderRadius.circular(10),
                  hoverColor: const Color(0xff00ff00),
                  onTap: () {},
                ),
              ),
            ),
          ),
        ),
      ),
    );
    await tester.pumpAndSettle();
    final RenderObject inkFeatures = getInkFeatures(tester);
    expect(inkFeatures, paintsExactlyCountTimes(#drawRRect, 0));

    // Hover the ink well.
    final TestGesture gesture = await tester.createGesture(
      kind: PointerDeviceKind.mouse,
      pointer: 1,
    );
    await gesture.addPointer(location: tester.getRect(find.byType(InkWell)).center);
    await tester.pumpAndSettle();
    expect(inkFeatures, paintsExactlyCountTimes(#drawRRect, 1));

    expect(
      inkFeatures,
      paints..rrect(
        rrect: RRect.fromLTRBR(350.0, 250.0, 450.0, 350.0, const Radius.circular(10)),
        color: const Color(0xff00ff00),
      ),
    );
  });

  testWidgets('InkWell customBorder clips for focus highlight', (WidgetTester tester) async {
    FocusManager.instance.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    final FocusNode focusNode = FocusNode(debugLabel: 'Ink Focus');
    await tester.pumpWidget(
      Material(
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: Align(
            alignment: Alignment.topLeft,
            child: SizedBox(
              width: 100,
              height: 100,
              child: MouseRegion(
                child: InkWell(
                  focusNode: focusNode,
                  borderRadius: BorderRadius.circular(10),
                  customBorder: const CircleBorder(),
                  hoverColor: const Color(0xff00ff00),
                  onTap: () {},
                ),
              ),
            ),
          ),
        ),
      ),
    );
    await tester.pumpAndSettle();
    final RenderObject inkFeatures = getInkFeatures(tester);
    expect(inkFeatures, paintsExactlyCountTimes(#clipPath, 0));
    expect(inkFeatures, paintsExactlyCountTimes(#drawRRect, 0));

    focusNode.requestFocus();
    await tester.pumpAndSettle();
    expect(inkFeatures, paintsExactlyCountTimes(#clipPath, 1));
    expect(inkFeatures, paintsExactlyCountTimes(#drawRRect, 1));

    // Create a rounded rectangle path with a radius that makes it similar to the custom border circle.
    const Rect expectedClipRect = Rect.fromLTRB(0, 0, 100, 100);
    final Path expectedClipPath =
        Path()..addRRect(RRect.fromRectAndRadius(expectedClipRect, const Radius.circular(50.0)));
    // The ink well custom border path should match the rounded rectangle path.
    expect(
      inkFeatures,
      paints..clipPath(
        pathMatcher: coversSameAreaAs(
          expectedClipPath,
          areaToCompare: expectedClipRect.inflate(20.0),
          sampleSize: 100,
        ),
      ),
    );
    focusNode.dispose();
  });

  testWidgets('InkWell customBorder clips for hover highlight', (WidgetTester tester) async {
    await tester.pumpWidget(
      Material(
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: Align(
            alignment: Alignment.topLeft,
            child: SizedBox(
              width: 100,
              height: 100,
              child: MouseRegion(
                child: InkWell(
                  borderRadius: BorderRadius.circular(10),
                  customBorder: const CircleBorder(),
                  hoverColor: const Color(0xff00ff00),
                  onTap: () {},
                ),
              ),
            ),
          ),
        ),
      ),
    );
    await tester.pumpAndSettle();
    final RenderObject inkFeatures = getInkFeatures(tester);
    expect(inkFeatures, paintsExactlyCountTimes(#clipPath, 0));
    expect(inkFeatures, paintsExactlyCountTimes(#drawRRect, 0));

    // Hover the ink well.
    final TestGesture gesture = await tester.createGesture(
      kind: PointerDeviceKind.mouse,
      pointer: 1,
    );
    await gesture.addPointer(location: tester.getRect(find.byType(InkWell)).center);
    await tester.pumpAndSettle();
    expect(inkFeatures, paintsExactlyCountTimes(#clipPath, 1));
    expect(inkFeatures, paintsExactlyCountTimes(#drawRRect, 1));

    // Create a rounded rectangle path with a radius that makes it similar to the custom border circle.
    const Rect expectedClipRect = Rect.fromLTRB(0, 0, 100, 100);
    final Path expectedClipPath =
        Path()..addRRect(RRect.fromRectAndRadius(expectedClipRect, const Radius.circular(50.0)));
    // The ink well custom border path should match the rounded rectangle path.
    expect(
      inkFeatures,
      paints..clipPath(
        pathMatcher: coversSameAreaAs(
          expectedClipPath,
          areaToCompare: expectedClipRect.inflate(20.0),
          sampleSize: 100,
        ),
      ),
    );
  });

  testWidgets('InkResponse radius can be updated', (WidgetTester tester) async {
    FocusManager.instance.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    final FocusNode focusNode = FocusNode(debugLabel: 'Ink Focus');
    Widget boilerplate(double radius) {
      return Material(
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: Center(
            child: SizedBox(
              width: 100,
              height: 100,
              child: InkResponse(
                focusNode: focusNode,
                radius: radius,
                focusColor: const Color(0xff0000ff),
                onTap: () {},
              ),
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(boilerplate(10));
    await tester.pumpAndSettle();
    final RenderObject inkFeatures = getInkFeatures(tester);
    expect(inkFeatures, paintsExactlyCountTimes(#drawCircle, 0));

    focusNode.requestFocus();
    await tester.pumpAndSettle();
    expect(inkFeatures, paintsExactlyCountTimes(#drawCircle, 1));
    expect(inkFeatures, paints..circle(radius: 10, color: const Color(0xff0000ff)));

    await tester.pumpWidget(boilerplate(20));
    await tester.pumpAndSettle();
    expect(inkFeatures, paintsExactlyCountTimes(#drawCircle, 1));
    expect(inkFeatures, paints..circle(radius: 20, color: const Color(0xff0000ff)));
    focusNode.dispose();
  });

  testWidgets('InkResponse highlightShape can be updated', (WidgetTester tester) async {
    FocusManager.instance.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    final FocusNode focusNode = FocusNode(debugLabel: 'Ink Focus');
    Widget boilerplate(BoxShape shape) {
      return Material(
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: Center(
            child: SizedBox(
              width: 100,
              height: 100,
              child: InkResponse(
                focusNode: focusNode,
                highlightShape: shape,
                borderRadius: BorderRadius.circular(10),
                focusColor: const Color(0xff0000ff),
                onTap: () {},
              ),
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(boilerplate(BoxShape.circle));
    await tester.pumpAndSettle();
    final RenderObject inkFeatures = getInkFeatures(tester);
    expect(inkFeatures, paintsExactlyCountTimes(#drawCircle, 0));
    expect(inkFeatures, paintsExactlyCountTimes(#drawRRect, 0));

    focusNode.requestFocus();
    await tester.pumpAndSettle();
    expect(inkFeatures, paintsExactlyCountTimes(#drawCircle, 1));
    expect(inkFeatures, paintsExactlyCountTimes(#drawRRect, 0));

    await tester.pumpWidget(boilerplate(BoxShape.rectangle));
    await tester.pumpAndSettle();
    expect(inkFeatures, paintsExactlyCountTimes(#drawCircle, 0));
    expect(inkFeatures, paintsExactlyCountTimes(#drawRRect, 1));
    focusNode.dispose();
  });

  testWidgets('InkWell borderRadius can be updated', (WidgetTester tester) async {
    FocusManager.instance.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    final FocusNode focusNode = FocusNode(debugLabel: 'Ink Focus');
    Widget boilerplate(BorderRadius borderRadius) {
      return Material(
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: Center(
            child: SizedBox(
              width: 100,
              height: 100,
              child: InkWell(
                focusNode: focusNode,
                borderRadius: borderRadius,
                focusColor: const Color(0xff0000ff),
                onTap: () {},
              ),
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(boilerplate(BorderRadius.circular(10)));
    await tester.pumpAndSettle();
    final RenderObject inkFeatures = getInkFeatures(tester);
    expect(inkFeatures, paintsExactlyCountTimes(#drawRRect, 0));

    focusNode.requestFocus();
    await tester.pumpAndSettle();
    expect(inkFeatures, paintsExactlyCountTimes(#drawRRect, 1));
    expect(
      inkFeatures,
      paints..rrect(
        rrect: RRect.fromLTRBR(350.0, 250.0, 450.0, 350.0, const Radius.circular(10)),
        color: const Color(0xff0000ff),
      ),
    );

    await tester.pumpWidget(boilerplate(BorderRadius.circular(30)));
    await tester.pumpAndSettle();
    expect(inkFeatures, paintsExactlyCountTimes(#drawRRect, 1));
    expect(
      inkFeatures,
      paints..rrect(
        rrect: RRect.fromLTRBR(350.0, 250.0, 450.0, 350.0, const Radius.circular(30)),
        color: const Color(0xff0000ff),
      ),
    );
    focusNode.dispose();
  });

  testWidgets('InkWell customBorder can be updated', (WidgetTester tester) async {
    FocusManager.instance.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    final FocusNode focusNode = FocusNode(debugLabel: 'Ink Focus');
    Widget boilerplate(BorderRadius borderRadius) {
      return Material(
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: Align(
            alignment: Alignment.topLeft,
            child: SizedBox(
              width: 100,
              height: 100,
              child: MouseRegion(
                child: InkWell(
                  focusNode: focusNode,
                  customBorder: RoundedRectangleBorder(borderRadius: borderRadius),
                  hoverColor: const Color(0xff00ff00),
                  onTap: () {},
                ),
              ),
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(boilerplate(BorderRadius.circular(20)));
    await tester.pumpAndSettle();
    final RenderObject inkFeatures = getInkFeatures(tester);
    expect(inkFeatures, paintsExactlyCountTimes(#clipPath, 0));

    focusNode.requestFocus();
    await tester.pumpAndSettle();
    expect(inkFeatures, paintsExactlyCountTimes(#clipPath, 1));

    const Rect expectedClipRect = Rect.fromLTRB(0, 0, 100, 100);
    Path expectedClipPath =
        Path()..addRRect(RRect.fromRectAndRadius(expectedClipRect, const Radius.circular(20)));
    expect(
      inkFeatures,
      paints..clipPath(
        pathMatcher: coversSameAreaAs(
          expectedClipPath,
          areaToCompare: expectedClipRect.inflate(20.0),
          sampleSize: 100,
        ),
      ),
    );

    await tester.pumpWidget(boilerplate(BorderRadius.circular(40)));
    await tester.pumpAndSettle();
    expectedClipPath =
        Path()..addRRect(RRect.fromRectAndRadius(expectedClipRect, const Radius.circular(40)));
    expect(
      inkFeatures,
      paints..clipPath(
        pathMatcher: coversSameAreaAs(
          expectedClipPath,
          areaToCompare: expectedClipRect.inflate(20.0),
          sampleSize: 100,
        ),
      ),
    );
    focusNode.dispose();
  });

  testWidgets('InkWell splash customBorder can be updated', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/121626.
    final FocusNode focusNode = FocusNode(debugLabel: 'Ink Focus');
    Widget boilerplate(BorderRadius borderRadius) {
      return MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Material(
          child: Directionality(
            textDirection: TextDirection.ltr,
            child: Align(
              alignment: Alignment.topLeft,
              child: SizedBox(
                width: 100,
                height: 100,
                child: MouseRegion(
                  child: InkWell(
                    focusNode: focusNode,
                    customBorder: RoundedRectangleBorder(borderRadius: borderRadius),
                    onTap: () {},
                  ),
                ),
              ),
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(boilerplate(BorderRadius.circular(20)));
    await tester.pumpAndSettle();

    final RenderObject inkFeatures = getInkFeatures(tester);
    expect(inkFeatures, paintsExactlyCountTimes(#clipPath, 0));

    final TestGesture gesture = await tester.startGesture(
      tester.getRect(find.byType(InkWell)).center,
    );
    await tester.pump(const Duration(milliseconds: 200)); // Unconfirmed splash is well underway.
    expect(inkFeatures, paintsExactlyCountTimes(#clipPath, 2)); // Splash and highlight.

    const Rect expectedClipRect = Rect.fromLTRB(0, 0, 100, 100);
    Path expectedClipPath =
        Path()..addRRect(RRect.fromRectAndRadius(expectedClipRect, const Radius.circular(20)));

    // Check that the splash and the highlight are correctly clipped.
    expect(
      inkFeatures,
      paints
        ..clipPath(
          pathMatcher: coversSameAreaAs(
            expectedClipPath,
            areaToCompare: expectedClipRect.inflate(20.0),
            sampleSize: 100,
          ),
        )
        ..clipPath(
          pathMatcher: coversSameAreaAs(
            expectedClipPath,
            areaToCompare: expectedClipRect.inflate(20.0),
            sampleSize: 100,
          ),
        ),
    );

    await tester.pumpWidget(boilerplate(BorderRadius.circular(40)));
    await tester.pumpAndSettle();
    expectedClipPath =
        Path()..addRRect(RRect.fromRectAndRadius(expectedClipRect, const Radius.circular(40)));

    // Check that the splash and the highlight are correctly clipped.
    expect(
      inkFeatures,
      paints
        ..clipPath(
          pathMatcher: coversSameAreaAs(
            expectedClipPath,
            areaToCompare: expectedClipRect.inflate(20.0),
            sampleSize: 100,
          ),
        )
        ..clipPath(
          pathMatcher: coversSameAreaAs(
            expectedClipPath,
            areaToCompare: expectedClipRect.inflate(20.0),
            sampleSize: 100,
          ),
        ),
    );

    await gesture.up();
    focusNode.dispose();
  });

  testWidgets("ink response doesn't change color on focus when on touch device", (
    WidgetTester tester,
  ) async {
    FocusManager.instance.highlightStrategy = FocusHighlightStrategy.alwaysTouch;
    final FocusNode focusNode = FocusNode(debugLabel: 'Ink Focus');
    await tester.pumpWidget(
      Material(
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: Center(
            child: SizedBox(
              width: 100,
              height: 100,
              child: InkWell(
                focusNode: focusNode,
                hoverColor: const Color(0xff00ff00),
                splashColor: const Color(0xffff0000),
                focusColor: const Color(0xff0000ff),
                highlightColor: const Color(0xf00fffff),
                onTap: () {},
                onLongPress: () {},
                onHover: (bool hover) {},
              ),
            ),
          ),
        ),
      ),
    );
    await tester.pumpAndSettle();
    final RenderObject inkFeatures = getInkFeatures(tester);
    expect(inkFeatures, paintsExactlyCountTimes(#drawRect, 0));
    focusNode.requestFocus();
    await tester.pumpAndSettle();
    expect(inkFeatures, paintsExactlyCountTimes(#drawRect, 0));
    focusNode.dispose();
  });

  testWidgets('InkWell.mouseCursor changes cursor on hover', (WidgetTester tester) async {
    final TestGesture gesture = await tester.createGesture(
      kind: PointerDeviceKind.mouse,
      pointer: 1,
    );
    await gesture.addPointer(location: const Offset(1, 1));

    // Test argument works
    await tester.pumpWidget(
      Material(
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: MouseRegion(
            cursor: SystemMouseCursors.forbidden,
            child: InkWell(mouseCursor: SystemMouseCursors.click, onTap: () {}),
          ),
        ),
      ),
    );

    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.click,
    );

    // Test default of InkWell()
    await tester.pumpWidget(
      Material(
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: MouseRegion(cursor: SystemMouseCursors.forbidden, child: InkWell(onTap: () {})),
        ),
      ),
    );

    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.click,
    );

    // Test disabled
    await tester.pumpWidget(
      const Material(
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: MouseRegion(cursor: SystemMouseCursors.forbidden, child: InkWell()),
        ),
      ),
    );

    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.basic,
    );

    // Test default of InkResponse()
    await tester.pumpWidget(
      Material(
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: MouseRegion(
            cursor: SystemMouseCursors.forbidden,
            child: InkResponse(onTap: () {}),
          ),
        ),
      ),
    );

    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.click,
    );

    // Test disabled
    await tester.pumpWidget(
      const Material(
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: MouseRegion(cursor: SystemMouseCursors.forbidden, child: InkResponse()),
        ),
      ),
    );

    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.basic,
    );
  });

  testWidgets('InkResponse containing selectable text changes mouse cursor when hovered', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/104595.
    await tester.pumpWidget(
      MaterialApp(
        home: SelectionArea(
          child: Material(child: InkResponse(onTap: () {}, child: const Text('button'))),
        ),
      ),
    );

    final TestGesture gesture = await tester.createGesture(
      kind: PointerDeviceKind.mouse,
      pointer: 1,
    );
    await gesture.addPointer(location: tester.getCenter(find.byType(Text)));

    await tester.pump();

    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.click,
    );
  });

  group('feedback', () {
    late FeedbackTester feedback;

    setUp(() {
      feedback = FeedbackTester();
    });

    tearDown(() {
      feedback.dispose();
    });

    testWidgets('enabled (default)', (WidgetTester tester) async {
      await tester.pumpWidget(
        Material(
          child: Directionality(
            textDirection: TextDirection.ltr,
            child: Center(child: InkWell(onTap: () {}, onLongPress: () {})),
          ),
        ),
      );
      await tester.tap(find.byType(InkWell), pointer: 1);
      await tester.pump(const Duration(seconds: 1));
      expect(feedback.clickSoundCount, 1);
      expect(feedback.hapticCount, 0);

      await tester.tap(find.byType(InkWell), pointer: 1);
      await tester.pump(const Duration(seconds: 1));
      expect(feedback.clickSoundCount, 2);
      expect(feedback.hapticCount, 0);

      await tester.longPress(find.byType(InkWell), pointer: 1);
      await tester.pump(const Duration(seconds: 1));
      expect(feedback.clickSoundCount, 2);
      expect(feedback.hapticCount, 1);
    });

    testWidgets('disabled', (WidgetTester tester) async {
      await tester.pumpWidget(
        Material(
          child: Directionality(
            textDirection: TextDirection.ltr,
            child: Center(child: InkWell(onTap: () {}, onLongPress: () {}, enableFeedback: false)),
          ),
        ),
      );
      await tester.tap(find.byType(InkWell), pointer: 1);
      await tester.pump(const Duration(seconds: 1));
      expect(feedback.clickSoundCount, 0);
      expect(feedback.hapticCount, 0);

      await tester.longPress(find.byType(InkWell), pointer: 1);
      await tester.pump(const Duration(seconds: 1));
      expect(feedback.clickSoundCount, 0);
      expect(feedback.hapticCount, 0);
    });
  });

  testWidgets('splashing survives scrolling when keep-alive is enabled', (
    WidgetTester tester,
  ) async {
    Future<void> runTest(bool keepAlive) async {
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(useMaterial3: false),
          home: Material(
            child: CompositedTransformFollower(
              // forces a layer, which makes the paints easier to separate out
              link: LayerLink(),
              child: ListView(
                addAutomaticKeepAlives: keepAlive,
                dragStartBehavior: DragStartBehavior.down,
                children: <Widget>[
                  SizedBox(height: 500.0, child: InkWell(onTap: () {}, child: const Placeholder())),
                  const SizedBox(height: 500.0),
                  const SizedBox(height: 500.0),
                ],
              ),
            ),
          ),
        ),
      );
      expect(
        tester.renderObject<RenderProxyBox>(find.byType(PhysicalModel)).child,
        isNot(paints..circle()),
      );
      await tester.tap(find.byType(InkWell));
      await tester.pump();
      await tester.pump(const Duration(milliseconds: 10));
      expect(
        tester.renderObject<RenderProxyBox>(find.byType(PhysicalModel)).child,
        paints..circle(),
      );
      await tester.drag(find.byType(ListView), const Offset(0.0, -1000.0));
      await tester.pump(const Duration(milliseconds: 10));
      await tester.drag(find.byType(ListView), const Offset(0.0, 1000.0));
      await tester.pump(const Duration(milliseconds: 10));
      expect(
        tester.renderObject<RenderProxyBox>(find.byType(PhysicalModel)).child,
        keepAlive ? (paints..circle()) : isNot(paints..circle()),
      );
    }

    await runTest(true);
    await runTest(false);
  });

  testWidgets('excludeFromSemantics', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Material(child: InkWell(onTap: () {}, child: const Text('Button'))),
      ),
    );
    expect(
      semantics,
      includesNodeWith(
        label: 'Button',
        actions: <SemanticsAction>[SemanticsAction.tap, SemanticsAction.focus],
      ),
    );

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Material(
          child: InkWell(onTap: () {}, excludeFromSemantics: true, child: const Text('Button')),
        ),
      ),
    );
    expect(
      semantics,
      isNot(
        includesNodeWith(
          label: 'Button',
          actions: <SemanticsAction>[SemanticsAction.tap, SemanticsAction.focus],
        ),
      ),
    );

    semantics.dispose();
  });

  testWidgets("ink response doesn't focus when disabled", (WidgetTester tester) async {
    FocusManager.instance.highlightStrategy = FocusHighlightStrategy.alwaysTouch;
    final FocusNode focusNode = FocusNode(debugLabel: 'Ink Focus');
    final GlobalKey childKey = GlobalKey();
    await tester.pumpWidget(
      Material(
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: InkWell(
            autofocus: true,
            onTap: () {},
            onLongPress: () {},
            onHover: (bool hover) {},
            focusNode: focusNode,
            child: Container(key: childKey),
          ),
        ),
      ),
    );
    await tester.pumpAndSettle();
    expect(focusNode.hasPrimaryFocus, isTrue);
    await tester.pumpWidget(
      Material(
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: InkWell(focusNode: focusNode, child: Container(key: childKey)),
        ),
      ),
    );
    await tester.pumpAndSettle();
    expect(focusNode.hasPrimaryFocus, isFalse);
    focusNode.dispose();
  });

  testWidgets('ink response accepts focus when disabled in directional navigation mode', (
    WidgetTester tester,
  ) async {
    FocusManager.instance.highlightStrategy = FocusHighlightStrategy.alwaysTouch;
    final FocusNode focusNode = FocusNode(debugLabel: 'Ink Focus');
    final GlobalKey childKey = GlobalKey();
    await tester.pumpWidget(
      Material(
        child: MediaQuery(
          data: const MediaQueryData(navigationMode: NavigationMode.directional),
          child: Directionality(
            textDirection: TextDirection.ltr,
            child: InkWell(
              autofocus: true,
              onTap: () {},
              onLongPress: () {},
              onHover: (bool hover) {},
              focusNode: focusNode,
              child: Container(key: childKey),
            ),
          ),
        ),
      ),
    );
    await tester.pumpAndSettle();
    expect(focusNode.hasPrimaryFocus, isTrue);
    await tester.pumpWidget(
      Material(
        child: MediaQuery(
          data: const MediaQueryData(navigationMode: NavigationMode.directional),
          child: Directionality(
            textDirection: TextDirection.ltr,
            child: InkWell(focusNode: focusNode, child: Container(key: childKey)),
          ),
        ),
      ),
    );
    await tester.pumpAndSettle();
    expect(focusNode.hasPrimaryFocus, isTrue);
    focusNode.dispose();
  });

  testWidgets("ink response doesn't hover when disabled", (WidgetTester tester) async {
    FocusManager.instance.highlightStrategy = FocusHighlightStrategy.alwaysTouch;
    final FocusNode focusNode = FocusNode(debugLabel: 'Ink Focus');
    final GlobalKey childKey = GlobalKey();
    bool hovering = false;
    await tester.pumpWidget(
      Material(
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: SizedBox(
            width: 100,
            height: 100,
            child: InkWell(
              autofocus: true,
              onTap: () {},
              onLongPress: () {},
              onHover: (bool value) {
                hovering = value;
              },
              focusNode: focusNode,
              child: SizedBox(key: childKey),
            ),
          ),
        ),
      ),
    );
    await tester.pumpAndSettle();
    expect(focusNode.hasPrimaryFocus, isTrue);
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    await gesture.moveTo(tester.getCenter(find.byKey(childKey)));
    await tester.pumpAndSettle();
    expect(hovering, isTrue);

    await tester.pumpWidget(
      Material(
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: SizedBox(
            width: 100,
            height: 100,
            child: InkWell(
              focusNode: focusNode,
              onHover: (bool value) {
                hovering = value;
              },
              child: SizedBox(key: childKey),
            ),
          ),
        ),
      ),
    );

    await tester.pumpAndSettle();
    expect(focusNode.hasPrimaryFocus, isFalse);
    focusNode.dispose();
  });

  testWidgets('When ink wells are nested, only the inner one is triggered by tap splash', (
    WidgetTester tester,
  ) async {
    final GlobalKey middleKey = GlobalKey();
    final GlobalKey innerKey = GlobalKey();
    Widget paddedInkWell({Key? key, Widget? child}) {
      return InkWell(
        key: key,
        onTap: () {},
        child: Padding(padding: const EdgeInsets.all(50), child: child),
      );
    }

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Material(
          child: Directionality(
            textDirection: TextDirection.ltr,
            child: Center(
              child: paddedInkWell(
                child: paddedInkWell(
                  key: middleKey,
                  child: paddedInkWell(key: innerKey, child: const SizedBox(width: 50, height: 50)),
                ),
              ),
            ),
          ),
        ),
      ),
    );
    final MaterialInkController material = Material.of(tester.element(find.byKey(innerKey)));

    // Press
    final TestGesture gesture = await tester.startGesture(
      tester.getCenter(find.byKey(innerKey)),
      pointer: 1,
    );
    await tester.pump(const Duration(milliseconds: 200));
    expect(material, paintsExactlyCountTimes(#drawCircle, 1));

    // Up
    await gesture.up();
    await tester.pumpAndSettle();
    expect(material, paintsNothing);

    // Press again
    await gesture.down(tester.getCenter(find.byKey(innerKey)));
    await tester.pump(const Duration(milliseconds: 200));
    expect(material, paintsExactlyCountTimes(#drawCircle, 1));

    // Cancel
    await gesture.cancel();
    await tester.pumpAndSettle();
    expect(material, paintsNothing);

    // Press again
    await gesture.down(tester.getCenter(find.byKey(innerKey)));
    await tester.pump(const Duration(milliseconds: 200));
    expect(material, paintsExactlyCountTimes(#drawCircle, 1));

    // Use a second pointer to press
    final TestGesture gesture2 = await tester.startGesture(
      tester.getCenter(find.byKey(innerKey)),
      pointer: 2,
    );
    await tester.pump(const Duration(milliseconds: 200));
    expect(material, paintsExactlyCountTimes(#drawCircle, 1));
    await gesture2.up();
  });

  testWidgets('Reparenting parent should allow both inkwells to show splash afterwards', (
    WidgetTester tester,
  ) async {
    final GlobalKey middleKey = GlobalKey();
    final GlobalKey innerKey = GlobalKey();
    Widget paddedInkWell({Key? key, Widget? child}) {
      return InkWell(
        key: key,
        onTap: () {},
        child: Padding(padding: const EdgeInsets.all(50), child: child),
      );
    }

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Material(
          child: Directionality(
            textDirection: TextDirection.ltr,
            child: Align(
              alignment: Alignment.topLeft,
              child: SizedBox(
                width: 200,
                height: 100,
                child: Row(
                  children: <Widget>[
                    paddedInkWell(key: middleKey, child: paddedInkWell(key: innerKey)),
                    const SizedBox(),
                  ],
                ),
              ),
            ),
          ),
        ),
      ),
    );
    final MaterialInkController material = Material.of(tester.element(find.byKey(innerKey)));

    // Press
    final TestGesture gesture1 = await tester.startGesture(
      tester.getCenter(find.byKey(innerKey)),
      pointer: 1,
    );
    await tester.pump(const Duration(milliseconds: 200));
    expect(material, paintsExactlyCountTimes(#drawCircle, 1));

    // Reparent parent
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Material(
          child: Directionality(
            textDirection: TextDirection.ltr,
            child: Align(
              alignment: Alignment.topLeft,
              child: SizedBox(
                width: 200,
                height: 100,
                child: Row(
                  children: <Widget>[paddedInkWell(key: innerKey), paddedInkWell(key: middleKey)],
                ),
              ),
            ),
          ),
        ),
      ),
    );

    // Up
    await gesture1.up();
    await tester.pumpAndSettle();
    expect(material, paintsNothing);

    // Press the previous parent
    await gesture1.down(tester.getCenter(find.byKey(middleKey)));
    await tester.pump(const Duration(milliseconds: 200));
    expect(material, paintsExactlyCountTimes(#drawCircle, 1));

    // Use a second pointer to press the previous child
    final TestGesture gesture2 = await tester.startGesture(
      tester.getCenter(find.byKey(innerKey)),
      pointer: 2,
    );
    await tester.pump(const Duration(milliseconds: 200));
    expect(material, paintsExactlyCountTimes(#drawCircle, 2));

    // Finish gesture to release resources.
    await gesture1.up();
    await gesture2.up();
    await tester.pumpAndSettle();
  });

  testWidgets('Parent inkwell does not block child inkwells from splashes', (
    WidgetTester tester,
  ) async {
    final GlobalKey middleKey = GlobalKey();
    final GlobalKey innerKey = GlobalKey();
    Widget paddedInkWell({Key? key, Widget? child}) {
      return InkWell(
        key: key,
        onTap: () {},
        child: Padding(padding: const EdgeInsets.all(50), child: child),
      );
    }

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Material(
          child: Directionality(
            textDirection: TextDirection.ltr,
            child: Center(
              child: paddedInkWell(
                child: paddedInkWell(
                  key: middleKey,
                  child: paddedInkWell(key: innerKey, child: const SizedBox(width: 50, height: 50)),
                ),
              ),
            ),
          ),
        ),
      ),
    );
    final MaterialInkController material = Material.of(tester.element(find.byKey(innerKey)));

    // Press middle
    final TestGesture gesture1 = await tester.startGesture(
      tester.getTopLeft(find.byKey(middleKey)) + const Offset(1, 1),
      pointer: 1,
    );
    await tester.pump(const Duration(milliseconds: 200));
    expect(material, paintsExactlyCountTimes(#drawCircle, 1));

    // Press inner
    final TestGesture gesture2 = await tester.startGesture(
      tester.getCenter(find.byKey(innerKey)),
      pointer: 2,
    );
    await tester.pump(const Duration(milliseconds: 200));
    expect(material, paintsExactlyCountTimes(#drawCircle, 2));

    // Finish gesture to release resources.
    await gesture1.up();
    await gesture2.up();
    await tester.pumpAndSettle();
  });

  testWidgets('Parent inkwell can count the number of pressed children to prevent splash', (
    WidgetTester tester,
  ) async {
    final GlobalKey parentKey = GlobalKey();
    final GlobalKey leftKey = GlobalKey();
    final GlobalKey rightKey = GlobalKey();
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Material(
          child: Directionality(
            textDirection: TextDirection.ltr,
            child: Center(
              child: SizedBox(
                width: 100,
                height: 100,
                child: InkWell(
                  key: parentKey,
                  onTap: () {},
                  child: Center(
                    child: SizedBox(
                      width: 100,
                      height: 50,
                      child: Row(
                        children: <Widget>[
                          SizedBox(
                            width: 50,
                            height: 50,
                            child: InkWell(key: leftKey, onTap: () {}),
                          ),
                          SizedBox(
                            width: 50,
                            height: 50,
                            child: InkWell(key: rightKey, onTap: () {}),
                          ),
                        ],
                      ),
                    ),
                  ),
                ),
              ),
            ),
          ),
        ),
      ),
    );
    final MaterialInkController material = Material.of(tester.element(find.byKey(leftKey)));

    final Offset parentPosition = tester.getTopLeft(find.byKey(parentKey)) + const Offset(1, 1);

    // Press left child
    final TestGesture gesture1 = await tester.startGesture(
      tester.getCenter(find.byKey(leftKey)),
      pointer: 1,
    );
    await tester.pump(const Duration(milliseconds: 200));
    expect(material, paintsExactlyCountTimes(#drawCircle, 1));

    // Press right child
    final TestGesture gesture2 = await tester.startGesture(
      tester.getCenter(find.byKey(rightKey)),
      pointer: 2,
    );
    await tester.pump(const Duration(milliseconds: 200));
    expect(material, paintsExactlyCountTimes(#drawCircle, 2));

    // Press parent
    final TestGesture gesture3 = await tester.startGesture(parentPosition, pointer: 3);
    await tester.pump(const Duration(milliseconds: 200));
    expect(material, paintsExactlyCountTimes(#drawCircle, 2));
    await gesture3.up();

    // Release left child
    await gesture1.up();
    await tester.pumpAndSettle();
    expect(material, paintsExactlyCountTimes(#drawCircle, 1));

    // Press parent
    await gesture3.down(parentPosition);
    await tester.pump(const Duration(milliseconds: 200));
    expect(material, paintsExactlyCountTimes(#drawCircle, 1));
    await gesture3.up();

    // Release right child
    await gesture2.up();
    await tester.pumpAndSettle();
    expect(material, paintsExactlyCountTimes(#drawCircle, 0));

    // Press parent
    await gesture3.down(parentPosition);
    await tester.pump(const Duration(milliseconds: 200));
    expect(material, paintsExactlyCountTimes(#drawCircle, 1));
    await gesture3.up();
  });

  testWidgets(
    'When ink wells are reparented, the old parent can display splash while the new parent can not',
    (WidgetTester tester) async {
      final GlobalKey innerKey = GlobalKey();
      final GlobalKey leftKey = GlobalKey();
      final GlobalKey rightKey = GlobalKey();

      Widget doubleInkWellRow({
        required double leftWidth,
        required double rightWidth,
        Widget? leftChild,
        Widget? rightChild,
      }) {
        return MaterialApp(
          theme: ThemeData(useMaterial3: false),
          home: Material(
            child: Directionality(
              textDirection: TextDirection.ltr,
              child: Align(
                alignment: Alignment.topLeft,
                child: SizedBox(
                  width: leftWidth + rightWidth,
                  height: 100,
                  child: Row(
                    children: <Widget>[
                      SizedBox(
                        width: leftWidth,
                        height: 100,
                        child: InkWell(
                          key: leftKey,
                          onTap: () {},
                          child: Center(
                            child: SizedBox(width: leftWidth, height: 50, child: leftChild),
                          ),
                        ),
                      ),
                      SizedBox(
                        width: rightWidth,
                        height: 100,
                        child: InkWell(
                          key: rightKey,
                          onTap: () {},
                          child: Center(
                            child: SizedBox(width: leftWidth, height: 50, child: rightChild),
                          ),
                        ),
                      ),
                    ],
                  ),
                ),
              ),
            ),
          ),
        );
      }

      await tester.pumpWidget(
        doubleInkWellRow(
          leftWidth: 110,
          rightWidth: 90,
          leftChild: InkWell(key: innerKey, onTap: () {}),
        ),
      );
      final MaterialInkController material = Material.of(tester.element(find.byKey(innerKey)));

      // Press inner
      final TestGesture gesture = await tester.startGesture(const Offset(100, 50), pointer: 1);
      await tester.pump(const Duration(milliseconds: 200));
      expect(material, paintsExactlyCountTimes(#drawCircle, 1));

      // Switch side
      await tester.pumpWidget(
        doubleInkWellRow(
          leftWidth: 90,
          rightWidth: 110,
          rightChild: InkWell(key: innerKey, onTap: () {}),
        ),
      );
      expect(material, paintsExactlyCountTimes(#drawCircle, 0));

      // A second pointer presses inner
      final TestGesture gesture2 = await tester.startGesture(const Offset(100, 50), pointer: 2);
      await tester.pump(const Duration(milliseconds: 200));
      expect(material, paintsExactlyCountTimes(#drawCircle, 1));

      await gesture.up();
      await gesture2.up();
      await tester.pumpAndSettle();

      // Press inner
      await gesture.down(const Offset(100, 50));
      await tester.pump(const Duration(milliseconds: 200));
      expect(material, paintsExactlyCountTimes(#drawCircle, 1));

      // Press left
      await gesture2.down(const Offset(50, 50));
      await tester.pump(const Duration(milliseconds: 200));
      expect(material, paintsExactlyCountTimes(#drawCircle, 2));

      await gesture.up();
      await gesture2.up();
    },
  );

  testWidgets(
    "Ink wells's splash starts before tap is confirmed and disappear after tap is canceled",
    (WidgetTester tester) async {
      final GlobalKey innerKey = GlobalKey();
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(useMaterial3: false),
          home: Material(
            child: Directionality(
              textDirection: TextDirection.ltr,
              child: GestureDetector(
                onHorizontalDragStart: (_) {},
                child: Center(
                  child: SizedBox(
                    width: 100,
                    height: 100,
                    child: InkWell(
                      onTap: () {},
                      child: Center(
                        child: SizedBox(
                          width: 50,
                          height: 50,
                          child: InkWell(key: innerKey, onTap: () {}),
                        ),
                      ),
                    ),
                  ),
                ),
              ),
            ),
          ),
        ),
      );
      final MaterialInkController material = Material.of(tester.element(find.byKey(innerKey)));

      // Press
      final TestGesture gesture = await tester.startGesture(
        tester.getCenter(find.byKey(innerKey)),
        pointer: 1,
      );
      await tester.pump(const Duration(milliseconds: 200));
      expect(material, paintsExactlyCountTimes(#drawCircle, 1));

      // Scroll upward
      await gesture.moveBy(const Offset(0, -100));
      await tester.pumpAndSettle();
      expect(material, paintsNothing);

      // Up
      await gesture.up();
      await tester.pumpAndSettle();
      expect(material, paintsNothing);

      // Press again
      await gesture.down(tester.getCenter(find.byKey(innerKey)));
      await tester.pump(const Duration(milliseconds: 200));
      expect(material, paintsExactlyCountTimes(#drawCircle, 1));
    },
  );

  testWidgets('disabled and hovered inkwell responds to mouse-exit', (WidgetTester tester) async {
    int onHoverCount = 0;
    late bool hover;

    Widget buildFrame({required bool enabled}) {
      return Material(
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: Center(
            child: SizedBox(
              width: 100,
              height: 100,
              child: InkWell(
                onTap: enabled ? () {} : null,
                onHover: (bool value) {
                  onHoverCount += 1;
                  hover = value;
                },
              ),
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildFrame(enabled: true));
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();

    await gesture.moveTo(tester.getCenter(find.byType(InkWell)));
    await tester.pumpAndSettle();
    expect(onHoverCount, 1);
    expect(hover, true);

    await tester.pumpWidget(buildFrame(enabled: false));
    await tester.pumpAndSettle();
    await gesture.moveTo(Offset.zero);
    // Even though the InkWell has been disabled, the mouse-exit still
    // causes onHover(false) to be called.
    expect(onHoverCount, 2);
    expect(hover, false);

    await gesture.moveTo(tester.getCenter(find.byType(InkWell)));
    await tester.pumpAndSettle();
    // We no longer see hover events because the InkWell is disabled
    // and it's no longer in the "hovering" state.
    expect(onHoverCount, 2);
    expect(hover, false);

    await tester.pumpWidget(buildFrame(enabled: true));
    await tester.pumpAndSettle();
    // The InkWell was enabled while it contained the mouse, however
    // we do not call onHover() because it may call setState().
    expect(onHoverCount, 2);
    expect(hover, false);

    await gesture.moveTo(tester.getCenter(find.byType(InkWell)) - const Offset(1, 1));
    await tester.pumpAndSettle();
    // Moving the mouse a little within the InkWell doesn't change anything.
    expect(onHoverCount, 2);
    expect(hover, false);
  });

  testWidgets('hovered ink well draws a transparent highlight when disabled', (
    WidgetTester tester,
  ) async {
    Widget buildFrame({required bool enabled}) {
      return Material(
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: Center(
            child: SizedBox(
              width: 100,
              height: 100,
              child: InkWell(
                onTap: enabled ? () {} : null,
                onHover: (bool value) {},
                hoverColor: const Color(0xff00ff00),
              ),
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildFrame(enabled: true));
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();

    // Hover the enabled InkWell.
    await gesture.moveTo(tester.getCenter(find.byType(InkWell)));
    await tester.pumpAndSettle();
    expect(
      find.byType(Material),
      paints..rect(
        color: const Color(0xff00ff00),
        rect: const Rect.fromLTRB(350.0, 250.0, 450.0, 350.0),
      ),
    );

    // Disable the hovered InkWell.
    await tester.pumpWidget(buildFrame(enabled: false));
    await tester.pumpAndSettle();
    expect(
      find.byType(Material),
      paints..rect(
        color: const Color(0x0000ff00),
        rect: const Rect.fromLTRB(350.0, 250.0, 450.0, 350.0),
      ),
    );
  });

  testWidgets('Changing InkWell.enabled should not trigger TextButton setState()', (
    WidgetTester tester,
  ) async {
    Widget buildFrame({required bool enabled}) {
      return Material(
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: Center(
            child: TextButton(onPressed: enabled ? () {} : null, child: const Text('button')),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildFrame(enabled: false));

    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    await gesture.moveTo(tester.getCenter(find.byType(TextButton)));
    await tester.pumpAndSettle();

    // Rebuilding the button with enabled:true causes InkWell.didUpdateWidget()
    // to be called per the change in its enabled flag. If onHover() was called,
    // this test would crash.
    await tester.pumpWidget(buildFrame(enabled: true));
    await tester.pumpAndSettle();

    // Rebuild again, with enabled:false
    await gesture.moveBy(const Offset(1, 1));
    await tester.pumpWidget(buildFrame(enabled: false));
    await tester.pumpAndSettle();
  });

  testWidgets(
    'InkWell does not attach semantics handler for onTap if it was not provided an onTap handler',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: Material(
            child: Center(child: InkWell(onLongPress: () {}, child: const Text('Foo'))),
          ),
        ),
      );

      expect(
        tester.getSemantics(find.bySemanticsLabel('Foo')),
        matchesSemantics(
          label: 'Foo',
          hasLongPressAction: true,
          isFocusable: true,
          hasFocusAction: true,
          textDirection: TextDirection.ltr,
        ),
      );

      // Add tap handler and confirm addition to semantic actions.
      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: Material(
            child: Center(
              child: InkWell(onLongPress: () {}, onTap: () {}, child: const Text('Foo')),
            ),
          ),
        ),
      );

      expect(
        tester.getSemantics(find.bySemanticsLabel('Foo')),
        matchesSemantics(
          label: 'Foo',
          hasTapAction: true,
          hasFocusAction: true,
          hasLongPressAction: true,
          isFocusable: true,
          textDirection: TextDirection.ltr,
        ),
      );
    },
  );

  testWidgets('InkWell highlight should not survive after [onTapDown, onDoubleTap] sequence', (
    WidgetTester tester,
  ) async {
    final List<String> log = <String>[];

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Material(
          child: Center(
            child: InkWell(
              onTap: () {
                log.add('tap');
              },
              onDoubleTap: () {
                log.add('double-tap');
              },
              onTapDown: (TapDownDetails details) {
                log.add('tap-down');
              },
              onTapCancel: () {
                log.add('tap-cancel');
              },
            ),
          ),
        ),
      ),
    );

    final Offset taplocation = tester.getRect(find.byType(InkWell)).center;

    final TestGesture gesture = await tester.startGesture(taplocation);
    await tester.pump(const Duration(milliseconds: 100));
    expect(log, equals(<String>['tap-down']));
    await gesture.up();
    await tester.pump(const Duration(milliseconds: 100));
    await tester.tap(find.byType(InkWell));
    await tester.pump(const Duration(milliseconds: 100));
    expect(log, equals(<String>['tap-down', 'double-tap']));

    await tester.pumpAndSettle();
    final RenderObject inkFeatures = getInkFeatures(tester);
    expect(inkFeatures, paintsExactlyCountTimes(#drawRect, 0));
  });

  testWidgets(
    'InkWell splash should not survive after [onTapDown, onTapDown, onTapCancel, onDoubleTap] sequence',
    (WidgetTester tester) async {
      final List<String> log = <String>[];

      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: Material(
            child: Center(
              child: InkWell(
                onTap: () {
                  log.add('tap');
                },
                onDoubleTap: () {
                  log.add('double-tap');
                },
                onTapDown: (TapDownDetails details) {
                  log.add('tap-down');
                },
                onTapCancel: () {
                  log.add('tap-cancel');
                },
              ),
            ),
          ),
        ),
      );

      final Offset tapLocation = tester.getRect(find.byType(InkWell)).center;

      final TestGesture gesture1 = await tester.startGesture(tapLocation);
      await tester.pump(const Duration(milliseconds: 100));
      expect(log, equals(<String>['tap-down']));
      await gesture1.up();
      await tester.pump(const Duration(milliseconds: 100));

      final TestGesture gesture2 = await tester.startGesture(tapLocation);
      await tester.pump(const Duration(milliseconds: 100));
      expect(log, equals(<String>['tap-down', 'tap-down']));
      await gesture2.up();
      await tester.pump(const Duration(milliseconds: 100));
      expect(log, equals(<String>['tap-down', 'tap-down', 'tap-cancel', 'double-tap']));

      await tester.pumpAndSettle();
      final RenderObject inkFeatures = getInkFeatures(tester);
      expect(inkFeatures, paintsExactlyCountTimes(#drawCircle, 0));
    },
  );

  testWidgets('InkWell disposes statesController', (WidgetTester tester) async {
    int tapCount = 0;
    Widget buildFrame(MaterialStatesController? statesController) {
      return MaterialApp(
        home: Scaffold(
          body: Center(
            child: InkWell(
              statesController: statesController,
              onTap: () {
                tapCount += 1;
              },
              child: const Text('inkwell'),
            ),
          ),
        ),
      );
    }

    final MaterialStatesController controller = MaterialStatesController();
    addTearDown(controller.dispose);
    int pressedCount = 0;
    controller.addListener(() {
      if (controller.value.contains(MaterialState.pressed)) {
        pressedCount += 1;
      }
    });

    await tester.pumpWidget(buildFrame(controller));
    await tester.tap(find.byType(InkWell));
    await tester.pumpAndSettle();
    expect(tapCount, 1);
    expect(pressedCount, 1);

    await tester.pumpWidget(buildFrame(null));
    await tester.tap(find.byType(InkWell));
    await tester.pumpAndSettle();
    expect(tapCount, 2);
    expect(pressedCount, 1);

    await tester.pumpWidget(buildFrame(controller));
    await tester.tap(find.byType(InkWell));
    await tester.pumpAndSettle();
    expect(tapCount, 3);
    expect(pressedCount, 2);
  });

  testWidgets('ink well overlayColor opacity fades from 0xff when hover ends', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/110266
    await tester.pumpWidget(
      Material(
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: Center(
            child: SizedBox(
              width: 100,
              height: 100,
              child: InkWell(
                overlayColor: MaterialStateProperty.resolveWith<Color?>((
                  Set<MaterialState> states,
                ) {
                  if (states.contains(MaterialState.hovered)) {
                    return const Color(0xff00ff00);
                  }
                  return null;
                }),
                onTap: () {},
                onLongPress: () {},
                onHover: (bool hover) {},
              ),
            ),
          ),
        ),
      ),
    );
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    await gesture.moveTo(tester.getCenter(find.byType(SizedBox)));
    await tester.pumpAndSettle();
    await gesture.moveTo(const Offset(10, 10)); // fade out the overlay
    await tester.pump(); // trigger the fade out animation
    final RenderObject inkFeatures = getInkFeatures(tester);
    // Fadeout begins with the MaterialStates.hovered overlay color
    expect(
      inkFeatures,
      paints..rect(
        rect: const Rect.fromLTRB(350.0, 250.0, 450.0, 350.0),
        color: const Color(0xff00ff00),
      ),
    );
    // 50ms fadeout is 50% complete, overlay color alpha goes from 0xff to 0x80
    await tester.pump(const Duration(milliseconds: 25));
    expect(
      inkFeatures,
      paints..rect(
        rect: const Rect.fromLTRB(350.0, 250.0, 450.0, 350.0),
        color: const Color(0x8000ff00),
      ),
    );
  });

  testWidgets('InkWell secondary tap test', (WidgetTester tester) async {
    final List<String> log = <String>[];

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Material(
          child: Center(
            child: InkWell(
              onSecondaryTap: () {
                log.add('secondary-tap');
              },
              onSecondaryTapDown: (TapDownDetails details) {
                log.add('secondary-tap-down');
              },
              onSecondaryTapUp: (TapUpDetails details) {
                log.add('secondary-tap-up');
              },
              onSecondaryTapCancel: () {
                log.add('secondary-tap-cancel');
              },
            ),
          ),
        ),
      ),
    );

    await tester.tap(find.byType(InkWell), pointer: 1, buttons: kSecondaryButton);

    expect(log, equals(<String>['secondary-tap-down', 'secondary-tap-up', 'secondary-tap']));
    log.clear();

    final TestGesture gesture = await tester.startGesture(
      tester.getCenter(find.byType(InkWell)),
      pointer: 2,
      buttons: kSecondaryButton,
    );
    await gesture.moveTo(const Offset(100, 100));
    await gesture.up();

    expect(log, equals(<String>['secondary-tap-down', 'secondary-tap-cancel']));
  });

  // Regression test for https://github.com/flutter/flutter/issues/124328.
  testWidgets(
    'InkWell secondary tap should not draw a splash when no secondary callbacks are defined',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: Material(child: Center(child: InkWell(onTap: () {}))),
        ),
      );

      final TestGesture gesture = await tester.startGesture(
        tester.getRect(find.byType(InkWell)).center,
        buttons: kSecondaryButton,
      );
      await tester.pump(const Duration(milliseconds: 200));

      // No splash should be painted.
      final RenderObject inkFeatures = getInkFeatures(tester);
      expect(inkFeatures, paintsExactlyCountTimes(#drawCircle, 0));

      await gesture.up();
    },
  );

  testWidgets('try out hoverDuration property', (WidgetTester tester) async {
    final List<String> log = <String>[];

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Material(
          child: Center(
            child: InkWell(
              hoverDuration: const Duration(milliseconds: 1000),
              onTap: () {
                log.add('tap');
              },
            ),
          ),
        ),
      ),
    );

    await tester.tap(find.byType(InkWell), pointer: 1);
    await tester.pump(const Duration(seconds: 1));

    expect(log, equals(<String>['tap']));
    log.clear();
  });

  testWidgets('InkWell activation action does not end immediately', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/132377.
    final MaterialStatesController controller = MaterialStatesController();

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Shortcuts(
          shortcuts: const <ShortcutActivator, Intent>{
            SingleActivator(LogicalKeyboardKey.enter): ButtonActivateIntent(),
          },
          child: Material(
            child: Center(
              child: InkWell(autofocus: true, onTap: () {}, statesController: controller),
            ),
          ),
        ),
      ),
    );

    // Invoke the InkWell activation action.
    await tester.sendKeyEvent(LogicalKeyboardKey.enter);

    // The InkWell is in pressed state.
    await tester.pump(const Duration(milliseconds: 99));
    expect(controller.value.contains(MaterialState.pressed), isTrue);

    await tester.pumpAndSettle();
    expect(controller.value.contains(MaterialState.pressed), isFalse);

    controller.dispose();
  });
}
