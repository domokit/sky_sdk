// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This file is run as part of a reduced test set in CI on Mac and Windows
// machines.
@Tags(<String>['reduced-test-set'])
library;

import 'dart:math';
import 'dart:ui';

import 'package:flutter/cupertino.dart';
import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

import '../widgets/semantics_tester.dart';

void main() {
  testWidgets('Overall appearance is correct for the light theme', (WidgetTester tester) async {
    await tester.pumpWidget(
      TestScaffoldApp(
        theme: const CupertinoThemeData(brightness: Brightness.light),
        dialog: CupertinoAlertDialog(
          content: const Text('The content'),
          actions: <Widget>[
            CupertinoDialogAction(child: const Text('One'), onPressed: () {}),
            CupertinoDialogAction(child: const Text('Two'), onPressed: () {}),
          ],
        ),
      ),
    );

    await tester.tap(find.text('Go'));
    await tester.pumpAndSettle();

    final TestGesture gesture = await tester.startGesture(tester.getCenter(find.text('One')));
    await tester.pumpAndSettle();
    // This golden file also verifies the structure of an alert dialog that
    // has a content, no title, and no overscroll for any sections (in contrast
    // to cupertinoAlertDialog.dark-theme.png).
    await expectLater(
      find.byType(CupertinoApp),
      matchesGoldenFile('cupertinoAlertDialog.overall-light-theme.png'),
    );

    await gesture.up();
  });

  testWidgets('Overall appearance is correct for the dark theme', (WidgetTester tester) async {
    await tester.pumpWidget(
      TestScaffoldApp(
        theme: const CupertinoThemeData(brightness: Brightness.dark),
        dialog: CupertinoAlertDialog(
          title: const Text('The title'),
          content: const Text('The content'),
          actions: List<Widget>.generate(
            20,
            (int i) => CupertinoDialogAction(onPressed: () {}, child: Text('Button $i')),
          ),
        ),
      ),
    );

    await tester.tap(find.text('Go'));
    await tester.pumpAndSettle();

    final TestGesture gesture = await tester.startGesture(tester.getCenter(find.text('Button 0')));
    await tester.pumpAndSettle();
    // This golden file also verifies the structure of an action sheet that
    // has both a message and a title, and an overscrolled action section (in
    // contrast to cupertinoAlertDialog.light-theme.png).
    await expectLater(
      find.byType(CupertinoApp),
      matchesGoldenFile('cupertinoAlertDialog.overall-dark-theme.png'),
    );

    await gesture.up();
  });

  testWidgets('Taps on button calls onPressed', (WidgetTester tester) async {
    bool didDelete = false;

    await tester.pumpWidget(
      createAppWithButtonThatLaunchesDialog(
        dialogBuilder: (BuildContext context) {
          return CupertinoAlertDialog(
            title: const Text('The title'),
            content: const Text('The content'),
            actions: <Widget>[
              const CupertinoDialogAction(child: Text('Cancel')),
              CupertinoDialogAction(
                isDestructiveAction: true,
                onPressed: () {
                  didDelete = true;
                  Navigator.pop(context);
                },
                child: const Text('Delete'),
              ),
            ],
          );
        },
      ),
    );

    await tester.tap(find.text('Go'));
    await tester.pump();

    expect(didDelete, isFalse);

    await tester.tap(find.text('Delete'));
    await tester.pump();

    expect(didDelete, isTrue);
    expect(find.text('Delete'), findsNothing);
  });

  testWidgets('Can tap after scrolling', (WidgetTester tester) async {
    int? wasPressed;
    await tester.pumpWidget(
      createAppWithButtonThatLaunchesDialog(
        dialogBuilder: (BuildContext context) {
          return CupertinoAlertDialog(
            actions: List<Widget>.generate(
              20,
              (int i) => CupertinoDialogAction(
                onPressed: () {
                  expect(wasPressed, null);
                  wasPressed = i;
                },
                child: Text('Button $i'),
              ),
            ),
          );
        },
      ),
    );

    await tester.tap(find.text('Go'));
    await tester.pumpAndSettle();
    expect(find.text('Button 19').hitTestable(), findsNothing);

    final TestGesture gesture = await tester.startGesture(tester.getCenter(find.text('Button 1')));
    await tester.pumpAndSettle();
    // The dragging gesture must be dispatched in at least two segments.
    // The first movement starts the gesture without setting a delta.
    await gesture.moveBy(const Offset(0, -20));
    await tester.pumpAndSettle();
    await gesture.moveBy(const Offset(0, -1000));
    await tester.pumpAndSettle();
    await gesture.up();
    await tester.pumpAndSettle();
    expect(find.text('Button 19').hitTestable(), findsOne);

    await tester.tap(find.text('Button 19'));
    await tester.pumpAndSettle();
    expect(wasPressed, 19);
  });

  testWidgets('Taps at the padding of buttons calls onPressed', (WidgetTester tester) async {
    // Ensures that the entire button responds to hit tests, not just the text
    // part.
    bool wasPressed = false;
    await tester.pumpWidget(
      createAppWithButtonThatLaunchesDialog(
        dialogBuilder: (BuildContext context) {
          return CupertinoAlertDialog(
            actions: <Widget>[
              CupertinoDialogAction(
                child: const Text('One'),
                onPressed: () {
                  expect(wasPressed, false);
                  wasPressed = true;
                  Navigator.pop(context);
                },
              ),
            ],
          );
        },
      ),
    );

    await tester.tap(find.text('Go'));

    await tester.pump();
    await tester.pump(const Duration(seconds: 1));

    expect(wasPressed, isFalse);

    await tester.tapAt(tester.getTopLeft(find.text('One')) - const Offset(20, 0));

    expect(wasPressed, isTrue);

    await tester.pump();
    await tester.pump(const Duration(seconds: 1));

    expect(find.text('One'), findsNothing);
  });

  testWidgets('Taps on a button can be slided to other buttons', (WidgetTester tester) async {
    int? pressed;
    await tester.pumpWidget(
      createAppWithButtonThatLaunchesDialog(
        dialogBuilder: (BuildContext context) {
          return CupertinoAlertDialog(
            actions: <Widget>[
              CupertinoDialogAction(
                child: const Text('One'),
                onPressed: () {
                  expect(pressed, null);
                  pressed = 1;
                  Navigator.pop(context);
                },
              ),
              CupertinoDialogAction(
                child: const Text('Two'),
                onPressed: () {
                  expect(pressed, null);
                  pressed = 2;
                  Navigator.pop(context);
                },
              ),
            ],
          );
        },
      ),
    );

    await tester.tap(find.text('Go'));
    await tester.pumpAndSettle();
    expect(pressed, null);

    final TestGesture gesture = await tester.startGesture(tester.getCenter(find.text('Two')));
    await tester.pumpAndSettle();

    await gesture.moveTo(tester.getCenter(find.text('One')));
    await tester.pumpAndSettle();
    await expectLater(
      find.byType(CupertinoAlertDialog),
      matchesGoldenFile('cupertinoAlertDialog.press-drag.png'),
    );

    await gesture.up();
    expect(pressed, 1);
    await tester.pumpAndSettle();
    expect(find.text('One'), findsNothing);
  });

  testWidgets('Taps on the content can be slided to other buttons', (WidgetTester tester) async {
    bool wasPressed = false;
    await tester.pumpWidget(
      createAppWithButtonThatLaunchesDialog(
        dialogBuilder: (BuildContext context) {
          return CupertinoAlertDialog(
            title: const Text('The title'),
            actions: <Widget>[
              CupertinoDialogAction(
                child: const Text('One'),
                onPressed: () {
                  expect(wasPressed, false);
                  wasPressed = true;
                  Navigator.pop(context);
                },
              ),
            ],
          );
        },
      ),
    );

    await tester.tap(find.text('Go'));
    await tester.pumpAndSettle();
    expect(wasPressed, false);

    final TestGesture gesture = await tester.startGesture(tester.getCenter(find.text('The title')));
    await tester.pumpAndSettle();

    await gesture.moveTo(tester.getCenter(find.text('One')));
    await tester.pumpAndSettle();
    await gesture.up();
    expect(wasPressed, true);
    await tester.pumpAndSettle();
    expect(find.text('One'), findsNothing);
  });

  testWidgets('Taps on the barrier can not be slided to buttons', (WidgetTester tester) async {
    bool wasPressed = false;
    await tester.pumpWidget(
      createAppWithButtonThatLaunchesDialog(
        dialogBuilder: (BuildContext context) {
          return CupertinoAlertDialog(
            title: const Text('The title'),
            actions: <Widget>[
              CupertinoDialogAction(
                child: const Text('Cancel'),
                onPressed: () {
                  expect(wasPressed, false);
                  wasPressed = true;
                  Navigator.pop(context);
                },
              ),
            ],
          );
        },
      ),
    );

    await tester.tap(find.text('Go'));
    await tester.pumpAndSettle();
    expect(wasPressed, false);

    // Press on the barrier.
    final TestGesture gesture = await tester.startGesture(const Offset(100, 100));
    await tester.pumpAndSettle();

    await gesture.moveTo(tester.getCenter(find.text('Cancel')));
    await tester.pumpAndSettle();
    await gesture.up();
    expect(wasPressed, false);
    await tester.pumpAndSettle();
    expect(find.text('Cancel'), findsOne);
  });

  testWidgets('Sliding taps can still yield to scrolling after horizontal movement', (
    WidgetTester tester,
  ) async {
    int? pressed;
    await tester.pumpWidget(
      createAppWithButtonThatLaunchesDialog(
        dialogBuilder: (BuildContext context) {
          return CupertinoAlertDialog(
            content: Text('Long message' * 200),
            actions: List<Widget>.generate(
              10,
              (int i) => CupertinoDialogAction(
                onPressed: () {
                  expect(pressed, null);
                  pressed = i;
                },
                child: Text('Button $i'),
              ),
            ),
          );
        },
      ),
    );

    await tester.tap(find.text('Go'));
    await tester.pumpAndSettle();

    // Starts on a button.
    final TestGesture gesture = await tester.startGesture(tester.getCenter(find.text('Button 0')));
    await tester.pumpAndSettle();
    // Move horizontally.
    await gesture.moveBy(const Offset(-10, 2));
    await gesture.moveBy(const Offset(-100, 2));
    await tester.pumpAndSettle();
    // Scroll up.
    await gesture.moveBy(const Offset(0, -40));
    await gesture.moveBy(const Offset(0, -1000));
    await tester.pumpAndSettle();
    // Stop scrolling.
    await gesture.up();
    await tester.pumpAndSettle();
    // The actions section should have been scrolled up and Button 9 is visible.
    await tester.tap(find.text('Button 9'));
    expect(pressed, 9);
  });

  testWidgets('Sliding taps is responsive even before the drag starts', (
    WidgetTester tester,
  ) async {
    int? pressed;
    await tester.pumpWidget(
      createAppWithButtonThatLaunchesDialog(
        dialogBuilder: (BuildContext context) {
          return CupertinoAlertDialog(
            content: Text('Long message' * 200),
            actions: List<Widget>.generate(
              10,
              (int i) => CupertinoDialogAction(
                onPressed: () {
                  expect(pressed, null);
                  pressed = i;
                },
                child: Text('Button $i'),
              ),
            ),
          );
        },
      ),
    );

    await tester.tap(find.text('Go'));
    await tester.pumpAndSettle();

    // Find the location right within the upper edge of button 1.
    final Offset start =
        tester.getTopLeft(find.widgetWithText(CupertinoDialogAction, 'Button 1')) +
        const Offset(30, 5);
    // Verify that the start location is within button 1.
    await tester.tapAt(start);
    expect(pressed, 1);
    pressed = null;

    final TestGesture gesture = await tester.startGesture(start);
    await tester.pumpAndSettle();
    // Move slightly upwards without starting the drag
    await gesture.moveBy(const Offset(0, -10));
    await tester.pumpAndSettle();
    // Stop scrolling.
    await gesture.up();
    await tester.pumpAndSettle();
    expect(pressed, 0);
  });

  testWidgets('Sliding taps only recognizes the primary pointer', (WidgetTester tester) async {
    int? pressed;
    await tester.pumpWidget(
      createAppWithButtonThatLaunchesDialog(
        dialogBuilder: (BuildContext context) {
          return CupertinoAlertDialog(
            title: const Text('The title'),
            actions: List<Widget>.generate(
              8,
              (int i) => CupertinoDialogAction(
                onPressed: () {
                  expect(pressed, null);
                  pressed = i;
                },
                child: Text('Button $i'),
              ),
            ),
          );
        },
      ),
    );

    await tester.tap(find.text('Go'));
    await tester.pumpAndSettle();

    // Start gesture 1 at button 0
    final TestGesture gesture1 = await tester.startGesture(tester.getCenter(find.text('Button 0')));
    await gesture1.moveBy(const Offset(0, 20)); // Starts the gesture
    await tester.pumpAndSettle();

    // Start gesture 2 at button 1.
    final TestGesture gesture2 = await tester.startGesture(tester.getCenter(find.text('Button 1')));
    await gesture2.moveBy(const Offset(0, 20)); // Starts the gesture
    await tester.pumpAndSettle();

    // Move gesture 1 to button 2 and release.
    await gesture1.moveTo(tester.getCenter(find.text('Button 2')));
    await tester.pumpAndSettle();
    await gesture1.up();
    await tester.pumpAndSettle();

    expect(pressed, 2);
    pressed = null;

    // Tap at button 3, which becomes the new primary pointer and is recognized.
    await tester.tap(find.text('Button 3'));
    await tester.pumpAndSettle();
    expect(pressed, 3);
    pressed = null;

    // Move gesture 2 to button 4 and release.
    await gesture2.moveTo(tester.getCenter(find.text('Button 4')));
    await tester.pumpAndSettle();
    await gesture2.up();
    await tester.pumpAndSettle();

    // Non-primary pointers should not be recognized.
    expect(pressed, null);
  });

  testWidgets('Non-primary pointers can trigger scroll', (WidgetTester tester) async {
    int? pressed;
    await tester.pumpWidget(
      createAppWithButtonThatLaunchesDialog(
        dialogBuilder: (BuildContext context) {
          return CupertinoAlertDialog(
            actions: List<Widget>.generate(
              12,
              (int i) => CupertinoDialogAction(
                onPressed: () {
                  expect(pressed, null);
                  pressed = i;
                },
                child: Text('Button $i'),
              ),
            ),
          );
        },
      ),
    );

    await tester.tap(find.text('Go'));
    await tester.pumpAndSettle();

    // Start gesture 1 at button 0
    final TestGesture gesture1 = await tester.startGesture(tester.getCenter(find.text('Button 0')));
    await tester.pumpAndSettle();

    expect(tester.getTopLeft(find.text('Button 11')).dy, greaterThan(400));

    // Start gesture 2 at button 1 and scrolls.
    final TestGesture gesture2 = await tester.startGesture(tester.getCenter(find.text('Button 1')));
    await gesture2.moveBy(const Offset(0, -20));
    await gesture2.moveBy(const Offset(0, -500));
    await tester.pumpAndSettle();

    expect(tester.getTopLeft(find.text('Button 11')).dy, lessThan(400));

    // Release gesture 1, which should not trigger any buttons.
    await gesture1.up();
    await tester.pumpAndSettle();

    expect(pressed, null);
  });

  testWidgets('Taps on legacy button calls onPressed and renders correctly', (
    WidgetTester tester,
  ) async {
    // Legacy buttons are implemented with [GestureDetector.onTap]. Apps that
    // use customized legacy buttons should continue to work.
    bool wasPressed = false;
    await tester.pumpWidget(
      createAppWithButtonThatLaunchesDialog(
        dialogBuilder: (BuildContext context) {
          return CupertinoAlertDialog(
            actions: <Widget>[
              LegacyAction(
                child: const Text('Legacy'),
                onPressed: () {
                  expect(wasPressed, false);
                  wasPressed = true;
                  Navigator.pop(context);
                },
              ),
              CupertinoDialogAction(child: const Text('One'), onPressed: () {}),
              CupertinoDialogAction(child: const Text('Two'), onPressed: () {}),
            ],
          );
        },
      ),
    );

    await tester.tap(find.text('Go'));
    await tester.pumpAndSettle();
    expect(wasPressed, isFalse);

    // Push the legacy button and hold for a while to activate the pressing effect.
    final TestGesture gesture = await tester.startGesture(tester.getCenter(find.text('Legacy')));
    await tester.pump(const Duration(seconds: 1));
    expect(wasPressed, isFalse);
    await expectLater(
      find.byType(CupertinoAlertDialog),
      matchesGoldenFile('cupertinoAlertDialog.legacyButton.png'),
    );

    await gesture.up();
    await tester.pumpAndSettle();
    expect(wasPressed, isTrue);
    expect(find.text('Legacy'), findsNothing);
  });

  testWidgets('Dialog not barrier dismissible by default', (WidgetTester tester) async {
    await tester.pumpWidget(createAppWithCenteredButton(const Text('Go')));

    final BuildContext context = tester.element(find.text('Go'));

    showCupertinoDialog<void>(
      context: context,
      builder: (BuildContext context) {
        return Container(
          width: 100.0,
          height: 100.0,
          alignment: Alignment.center,
          child: const Text('Dialog'),
        );
      },
    );

    await tester.pumpAndSettle(const Duration(seconds: 1));
    expect(find.text('Dialog'), findsOneWidget);

    // Tap on the barrier, which shouldn't do anything this time.
    await tester.tapAt(const Offset(10.0, 10.0));

    await tester.pumpAndSettle(const Duration(seconds: 1));
    expect(find.text('Dialog'), findsOneWidget);
  });

  testWidgets('Dialog configurable to be barrier dismissible', (WidgetTester tester) async {
    await tester.pumpWidget(createAppWithCenteredButton(const Text('Go')));

    final BuildContext context = tester.element(find.text('Go'));

    showCupertinoDialog<void>(
      context: context,
      barrierDismissible: true,
      builder: (BuildContext context) {
        return Container(
          width: 100.0,
          height: 100.0,
          alignment: Alignment.center,
          child: const Text('Dialog'),
        );
      },
    );

    await tester.pumpAndSettle(const Duration(seconds: 1));
    expect(find.text('Dialog'), findsOneWidget);

    // Tap off the barrier.
    await tester.tapAt(const Offset(10.0, 10.0));

    await tester.pumpAndSettle(const Duration(seconds: 1));
    expect(find.text('Dialog'), findsNothing);
  });

  testWidgets('Dialog destructive action style', (WidgetTester tester) async {
    await tester.pumpWidget(
      boilerplate(const CupertinoDialogAction(isDestructiveAction: true, child: Text('Ok'))),
    );

    final DefaultTextStyle widget = tester.widget(find.byType(DefaultTextStyle));

    expect(widget.style.color!.withAlpha(255), CupertinoColors.systemRed.color);
  });

  testWidgets('Dialog default action style', (WidgetTester tester) async {
    await tester.pumpWidget(
      CupertinoTheme(
        data: const CupertinoThemeData(primaryColor: CupertinoColors.systemGreen),
        child: boilerplate(const CupertinoDialogAction(child: Text('Ok'))),
      ),
    );

    final DefaultTextStyle widget = tester.widget(find.byType(DefaultTextStyle));

    expect(widget.style.color!.withAlpha(255), CupertinoColors.systemGreen.color);
    expect(widget.style.fontFamily, 'CupertinoSystemText');
  });

  testWidgets('Dialog dark theme', (WidgetTester tester) async {
    await tester.pumpWidget(
      CupertinoApp(
        home: MediaQuery(
          data: const MediaQueryData(platformBrightness: Brightness.dark),
          child: CupertinoAlertDialog(
            title: const Text('The Title'),
            content: const Text('Content'),
            actions: <Widget>[
              CupertinoDialogAction(
                isDefaultAction: true,
                onPressed: () {},
                child: const Text('Cancel'),
              ),
              const CupertinoDialogAction(child: Text('OK')),
            ],
          ),
        ),
      ),
    );

    final RichText cancelText = tester.widget<RichText>(
      find.descendant(of: find.text('Cancel'), matching: find.byType(RichText)),
    );

    expect(
      cancelText.text.style!.color!.value,
      0xFF0A84FF, // dark elevated color of systemBlue.
    );

    expect(find.byType(CupertinoAlertDialog), paints..rect(color: const Color(0xCC2D2D2D)));
  });

  testWidgets('Has semantic annotations', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);
    await tester.pumpWidget(
      const CupertinoApp(
        home: CupertinoAlertDialog(
          title: Text('The Title'),
          content: Text('Content'),
          actions: <Widget>[
            CupertinoDialogAction(child: Text('Cancel')),
            CupertinoDialogAction(child: Text('OK')),
          ],
        ),
      ),
    );

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics(
              children: <TestSemantics>[
                TestSemantics(
                  children: <TestSemantics>[
                    TestSemantics(
                      flags: <SemanticsFlag>[SemanticsFlag.scopesRoute],
                      children: <TestSemantics>[
                        TestSemantics(
                          flags: <SemanticsFlag>[
                            SemanticsFlag.scopesRoute,
                            SemanticsFlag.namesRoute,
                          ],
                          label: 'Alert',
                          children: <TestSemantics>[
                            TestSemantics(
                              flags: <SemanticsFlag>[SemanticsFlag.hasImplicitScrolling],
                              children: <TestSemantics>[
                                TestSemantics(label: 'The Title'),
                                TestSemantics(label: 'Content'),
                              ],
                            ),
                            TestSemantics(
                              flags: <SemanticsFlag>[SemanticsFlag.hasImplicitScrolling],
                              children: <TestSemantics>[
                                TestSemantics(
                                  flags: <SemanticsFlag>[SemanticsFlag.isButton],
                                  label: 'Cancel',
                                ),
                                TestSemantics(
                                  flags: <SemanticsFlag>[SemanticsFlag.isButton],
                                  label: 'OK',
                                ),
                              ],
                            ),
                          ],
                        ),
                      ],
                    ),
                  ],
                ),
              ],
            ),
          ],
        ),
        ignoreId: true,
        ignoreRect: true,
        ignoreTransform: true,
      ),
    );

    semantics.dispose();
  });

  testWidgets('Dialog default action style', (WidgetTester tester) async {
    await tester.pumpWidget(
      boilerplate(const CupertinoDialogAction(isDefaultAction: true, child: Text('Ok'))),
    );

    final DefaultTextStyle widget = tester.widget(find.byType(DefaultTextStyle));

    expect(widget.style.fontWeight, equals(FontWeight.w600));
  });

  testWidgets('Dialog default and destructive action styles', (WidgetTester tester) async {
    await tester.pumpWidget(
      boilerplate(
        const CupertinoDialogAction(
          isDefaultAction: true,
          isDestructiveAction: true,
          child: Text('Ok'),
        ),
      ),
    );

    final DefaultTextStyle widget = tester.widget(find.byType(DefaultTextStyle));

    expect(widget.style.color!.withAlpha(255), CupertinoColors.systemRed.color);
    expect(widget.style.fontWeight, equals(FontWeight.w600));
  });

  testWidgets('Dialog disabled action style', (WidgetTester tester) async {
    await tester.pumpWidget(boilerplate(const CupertinoDialogAction(child: Text('Ok'))));

    final DefaultTextStyle widget = tester.widget(find.byType(DefaultTextStyle));

    expect(widget.style.color!.opacity, greaterThanOrEqualTo(127 / 255));
    expect(widget.style.color!.opacity, lessThanOrEqualTo(128 / 255));
  });

  testWidgets('Dialog enabled action style', (WidgetTester tester) async {
    await tester.pumpWidget(
      boilerplate(CupertinoDialogAction(child: const Text('Ok'), onPressed: () {})),
    );

    final DefaultTextStyle widget = tester.widget(find.byType(DefaultTextStyle));

    expect(widget.style.color!.opacity, equals(1.0));
  });

  testWidgets('Pressing on disabled buttons does not trigger highlight', (
    WidgetTester tester,
  ) async {
    bool pressedEnable = false;
    await tester.pumpWidget(
      createAppWithButtonThatLaunchesDialog(
        dialogBuilder: (BuildContext context) {
          return CupertinoAlertDialog(
            actions: <Widget>[
              const CupertinoDialogAction(child: Text('Disabled')),
              CupertinoDialogAction(
                isDestructiveAction: true,
                onPressed: () {
                  pressedEnable = true;
                  Navigator.pop(context);
                },
                child: const Text('Enabled'),
              ),
            ],
          );
        },
      ),
    );

    await tester.tap(find.text('Go'));

    await tester.pump();
    await tester.pump(const Duration(seconds: 1));

    final TestGesture gesture = await tester.startGesture(tester.getCenter(find.text('Disabled')));

    await tester.pumpAndSettle(const Duration(seconds: 1));

    // This should look exactly like an idle dialog.
    await expectLater(
      find.byType(CupertinoAlertDialog),
      matchesGoldenFile('cupertinoAlertDialog.press_disabled.png'),
    );

    // Verify that gestures that started on a disabled button can slide onto an
    // enabled button.
    await gesture.moveTo(tester.getCenter(find.text('Enabled')));
    await tester.pumpAndSettle();

    await expectLater(
      find.byType(CupertinoAlertDialog),
      matchesGoldenFile('cupertinoAlertDialog.press_disabled_slide_to_enabled.png'),
    );

    expect(pressedEnable, false);
    await gesture.up();
    expect(pressedEnable, true);
  });

  testWidgets('Action buttons shows pressed highlight as soon as the pointer is down', (
    WidgetTester tester,
  ) async {
    // Verifies that the the pressed color is not delayed for some milliseconds,
    // a symptom if the color relies on a tap gesture timing out.
    await tester.pumpWidget(
      createAppWithButtonThatLaunchesDialog(
        dialogBuilder: (BuildContext context) {
          return CupertinoAlertDialog(
            title: const Text('The title'),
            actions: <Widget>[
              CupertinoDialogAction(child: const Text('One'), onPressed: () {}),
              CupertinoDialogAction(child: const Text('Two'), onPressed: () {}),
            ],
          );
        },
      ),
    );

    await tester.tap(find.text('Go'));
    await tester.pumpAndSettle();

    final TestGesture pointer = await tester.startGesture(tester.getCenter(find.text('Two')));
    // Just `pump`, not `pumpAndSettle`, as we want to verify the very next frame.
    await tester.pump();
    await expectLater(
      find.byType(CupertinoAlertDialog),
      matchesGoldenFile('cupertinoAlertDialog.pressed.png'),
    );
    await pointer.up();
  });

  testWidgets('Message is scrollable, has correct padding with large text sizes', (
    WidgetTester tester,
  ) async {
    final ScrollController scrollController = ScrollController();
    addTearDown(scrollController.dispose);
    await tester.pumpWidget(
      createAppWithButtonThatLaunchesDialog(
        dialogBuilder: (BuildContext context) {
          return MediaQuery.withClampedTextScaling(
            minScaleFactor: 3.0,
            maxScaleFactor: 3.0,
            child: CupertinoAlertDialog(
              title: const Text('The Title'),
              content: Text('Very long content ' * 20),
              actions: const <Widget>[
                CupertinoDialogAction(child: Text('Cancel')),
                CupertinoDialogAction(isDestructiveAction: true, child: Text('OK')),
              ],
              scrollController: scrollController,
            ),
          );
        },
      ),
    );

    await tester.tap(find.text('Go'));
    await tester.pumpAndSettle();

    expect(scrollController.offset, 0.0);
    scrollController.jumpTo(100.0);
    expect(scrollController.offset, 100.0);
    // Set the scroll position back to zero.
    scrollController.jumpTo(0.0);

    await tester.pumpAndSettle();

    // Expect the modal dialog box to take all available height.
    expect(tester.getSize(find.byType(ClipRRect)), equals(const Size(310.0, 560.0 - 24.0 * 2)));

    // Check sizes/locations of the text. The text is large so these 2 buttons are stacked.
    // Visually the "Cancel" button and "OK" button are the same height when using the
    // regular font. However, when using the test font, "Cancel" becomes 2 lines which
    // is why the height we're verifying for "Cancel" is larger than "OK".

    if (!kIsWeb || isSkiaWeb) {
      // https://github.com/flutter/flutter/issues/99933
      expect(tester.getSize(find.text('The Title')), equals(const Size(270.0, 132.0)));
    }
    expect(tester.getTopLeft(find.text('The Title')), equals(const Offset(265.0, 80.0 + 24.0)));
    expect(
      tester.getSize(find.widgetWithText(CupertinoDialogAction, 'Cancel')),
      equals(const Size(310.0, 148.0)),
    );
    expect(
      tester.getSize(find.widgetWithText(CupertinoDialogAction, 'OK')),
      equals(const Size(310.0, 98.0)),
    );
  });

  testWidgets('Dialog respects small constraints.', (WidgetTester tester) async {
    final ScrollController scrollController = ScrollController();
    addTearDown(scrollController.dispose);
    await tester.pumpWidget(
      createAppWithButtonThatLaunchesDialog(
        dialogBuilder: (BuildContext context) {
          return Center(
            child: ConstrainedBox(
              // Constrain the dialog to a tiny size and ensure it respects
              // these exact constraints.
              constraints: BoxConstraints.tight(const Size(200.0, 100.0)),
              child: CupertinoAlertDialog(
                title: const Text('The Title'),
                content: const Text('The message'),
                actions: const <Widget>[
                  CupertinoDialogAction(child: Text('Option 1')),
                  CupertinoDialogAction(child: Text('Option 2')),
                  CupertinoDialogAction(child: Text('Option 3')),
                ],
                scrollController: scrollController,
              ),
            ),
          );
        },
      ),
    );

    await tester.tap(find.text('Go'));
    await tester.pump();

    const double topAndBottomMargin = 40.0;
    const double topAndBottomPadding = 24.0 * 2;
    const double leftAndRightPadding = 40.0 * 2;
    final Finder modalFinder = find.byType(ClipRRect);
    expect(
      tester.getSize(modalFinder),
      equals(
        const Size(200.0 - leftAndRightPadding, 100.0 - topAndBottomMargin - topAndBottomPadding),
      ),
    );
  });

  testWidgets('Button list is scrollable, has correct position with large text sizes.', (
    WidgetTester tester,
  ) async {
    final ScrollController actionScrollController = ScrollController();
    addTearDown(actionScrollController.dispose);
    await tester.pumpWidget(
      createAppWithButtonThatLaunchesDialog(
        dialogBuilder: (BuildContext context) {
          return MediaQuery.withClampedTextScaling(
            minScaleFactor: 3.0,
            maxScaleFactor: 3.0,
            child: CupertinoAlertDialog(
              title: const Text('The title'),
              content: const Text('The content.'),
              actions: const <Widget>[
                CupertinoDialogAction(child: Text('One')),
                CupertinoDialogAction(child: Text('Two')),
                CupertinoDialogAction(child: Text('Three')),
                CupertinoDialogAction(child: Text('Chocolate Brownies')),
                CupertinoDialogAction(isDestructiveAction: true, child: Text('Cancel')),
              ],
              actionScrollController: actionScrollController,
            ),
          );
        },
      ),
    );

    await tester.tap(find.text('Go'));

    await tester.pump();

    // Check that the action buttons list is scrollable.
    expect(actionScrollController.offset, 0.0);
    actionScrollController.jumpTo(100.0);
    expect(actionScrollController.offset, 100.0);
    actionScrollController.jumpTo(0.0);

    // Check that the action buttons are aligned vertically.
    expect(tester.getCenter(find.widgetWithText(CupertinoDialogAction, 'One')).dx, equals(400.0));
    expect(tester.getCenter(find.widgetWithText(CupertinoDialogAction, 'Two')).dx, equals(400.0));
    expect(tester.getCenter(find.widgetWithText(CupertinoDialogAction, 'Three')).dx, equals(400.0));
    expect(
      tester.getCenter(find.widgetWithText(CupertinoDialogAction, 'Chocolate Brownies')).dx,
      equals(400.0),
    );
    expect(
      tester.getCenter(find.widgetWithText(CupertinoDialogAction, 'Cancel')).dx,
      equals(400.0),
    );

    // Check that the action buttons are the correct heights.
    expect(tester.getSize(find.widgetWithText(CupertinoDialogAction, 'One')).height, equals(98.0));
    expect(tester.getSize(find.widgetWithText(CupertinoDialogAction, 'Two')).height, equals(98.0));
    expect(
      tester.getSize(find.widgetWithText(CupertinoDialogAction, 'Three')).height,
      equals(98.0),
    );
    expect(
      tester.getSize(find.widgetWithText(CupertinoDialogAction, 'Chocolate Brownies')).height,
      equals(248.0),
    );
    expect(
      tester.getSize(find.widgetWithText(CupertinoDialogAction, 'Cancel')).height,
      equals(148.0),
    );
  });

  testWidgets('Title Section is empty, Button section is not empty.', (WidgetTester tester) async {
    final ScrollController actionScrollController = ScrollController();
    addTearDown(actionScrollController.dispose);
    await tester.pumpWidget(
      createAppWithButtonThatLaunchesDialog(
        dialogBuilder: (BuildContext context) {
          return MediaQuery.withNoTextScaling(
            child: CupertinoAlertDialog(
              actions: const <Widget>[
                CupertinoDialogAction(child: Text('One')),
                CupertinoDialogAction(child: Text('Two')),
              ],
              actionScrollController: actionScrollController,
            ),
          );
        },
      ),
    );

    await tester.tap(find.text('Go'));

    await tester.pump();

    // Check that the dialog size is the same as the actions section size. This
    // ensures that an empty content section doesn't accidentally render some
    // empty space in the dialog.
    final Finder contentSectionFinder = find.byElementPredicate((Element element) {
      return element.widget.runtimeType.toString() == '_CupertinoAlertActionSection';
    });

    final Finder modalBoundaryFinder = find.byType(ClipRRect);

    expect(tester.getSize(contentSectionFinder), tester.getSize(modalBoundaryFinder));

    // Check that the title/message section is not displayed
    expect(actionScrollController.offset, 0.0);
    expect(tester.getTopLeft(find.widgetWithText(CupertinoDialogAction, 'One')).dy, equals(270.75));

    // Check that the button's vertical size is the same.
    expect(
      tester.getSize(find.widgetWithText(CupertinoDialogAction, 'One')).height,
      equals(tester.getSize(find.widgetWithText(CupertinoDialogAction, 'Two')).height),
    );
  });

  testWidgets('Button section is empty, Title section is not empty.', (WidgetTester tester) async {
    final ScrollController scrollController = ScrollController();
    addTearDown(scrollController.dispose);
    await tester.pumpWidget(
      createAppWithButtonThatLaunchesDialog(
        dialogBuilder: (BuildContext context) {
          return MediaQuery.withNoTextScaling(
            child: CupertinoAlertDialog(
              title: const Text('The title'),
              content: const Text('The content.'),
              scrollController: scrollController,
            ),
          );
        },
      ),
    );

    await tester.tap(find.text('Go'));

    await tester.pump();

    // Check that there's no button action section.
    expect(scrollController.offset, 0.0);
    expect(find.widgetWithText(CupertinoDialogAction, 'One'), findsNothing);

    // Check that the dialog size is the same as the content section size. This
    // ensures that an empty button section doesn't accidentally render some
    // empty space in the dialog.
    final Finder contentSectionFinder = find.byElementPredicate((Element element) {
      return element.widget.runtimeType.toString() == '_CupertinoAlertContentSection';
    });

    final Finder modalBoundaryFinder = find.byType(ClipRRect);

    expect(tester.getSize(contentSectionFinder), tester.getSize(modalBoundaryFinder));
  });

  testWidgets('Actions section height for 1 button is height of button.', (
    WidgetTester tester,
  ) async {
    final ScrollController scrollController = ScrollController();
    addTearDown(scrollController.dispose);
    await tester.pumpWidget(
      createAppWithButtonThatLaunchesDialog(
        dialogBuilder: (BuildContext context) {
          return CupertinoAlertDialog(
            title: const Text('The Title'),
            content: const Text('The message'),
            actions: const <Widget>[CupertinoDialogAction(child: Text('OK'))],
            scrollController: scrollController,
          );
        },
      ),
    );

    await tester.tap(find.text('Go'));
    await tester.pump();

    final RenderBox okButtonBox = findActionButtonRenderBoxByTitle(tester, 'OK');
    final RenderBox actionsSectionBox = findScrollableActionsSectionRenderBox(tester);

    expect(okButtonBox.size.width, actionsSectionBox.size.width);
    expect(okButtonBox.size.height, actionsSectionBox.size.height);
  });

  testWidgets('Actions section height for 2 side-by-side buttons is height of tallest button.', (
    WidgetTester tester,
  ) async {
    final ScrollController scrollController = ScrollController();
    addTearDown(scrollController.dispose);
    late double dividerWidth; // Will be set when the dialog builder runs. Needs a BuildContext.
    await tester.pumpWidget(
      createAppWithButtonThatLaunchesDialog(
        dialogBuilder: (BuildContext context) {
          dividerWidth = 0.3;
          return CupertinoAlertDialog(
            title: const Text('The Title'),
            content: const Text('The message'),
            actions: const <Widget>[
              CupertinoDialogAction(child: Text('OK')),
              CupertinoDialogAction(isDestructiveAction: true, child: Text('Cancel')),
            ],
            scrollController: scrollController,
          );
        },
      ),
    );

    await tester.tap(find.text('Go'));
    await tester.pump();

    final RenderBox okButtonBox = findActionButtonRenderBoxByTitle(tester, 'OK');
    final RenderBox cancelButtonBox = findActionButtonRenderBoxByTitle(tester, 'Cancel');
    final RenderBox actionsSectionBox = findScrollableActionsSectionRenderBox(tester);

    expect(okButtonBox.size.width, cancelButtonBox.size.width);

    expect(
      actionsSectionBox.size.width,
      okButtonBox.size.width + cancelButtonBox.size.width + dividerWidth,
    );

    expect(
      actionsSectionBox.size.height,
      max(okButtonBox.size.height, cancelButtonBox.size.height),
    );
  });

  testWidgets(
    'Actions section height for 2 stacked buttons with enough room is height of both buttons.',
    (WidgetTester tester) async {
      final ScrollController scrollController = ScrollController();
      addTearDown(scrollController.dispose);
      const double dividerThickness = 0.3;
      await tester.pumpWidget(
        createAppWithButtonThatLaunchesDialog(
          dialogBuilder: (BuildContext context) {
            return CupertinoAlertDialog(
              title: const Text('The Title'),
              content: const Text('The message'),
              actions: const <Widget>[
                CupertinoDialogAction(child: Text('OK')),
                CupertinoDialogAction(
                  isDestructiveAction: true,
                  child: Text('This is too long to fit'),
                ),
              ],
              scrollController: scrollController,
            );
          },
        ),
      );

      await tester.tap(find.text('Go'));
      await tester.pump();

      final RenderBox okButtonBox = findActionButtonRenderBoxByTitle(tester, 'OK');
      final RenderBox longButtonBox = findActionButtonRenderBoxByTitle(
        tester,
        'This is too long to fit',
      );
      final RenderBox actionsSectionBox = findScrollableActionsSectionRenderBox(tester);

      expect(okButtonBox.size.width, longButtonBox.size.width);

      expect(okButtonBox.size.width, actionsSectionBox.size.width);

      expect(
        okButtonBox.size.height + dividerThickness + longButtonBox.size.height,
        actionsSectionBox.size.height,
      );
    },
  );

  testWidgets(
    'Actions section height for 2 stacked buttons without enough room and regular font is 1.5 buttons tall.',
    (WidgetTester tester) async {
      final ScrollController scrollController = ScrollController();
      addTearDown(scrollController.dispose);
      await tester.pumpWidget(
        createAppWithButtonThatLaunchesDialog(
          dialogBuilder: (BuildContext context) {
            return CupertinoAlertDialog(
              title: const Text('The Title'),
              content: Text('The message\n' * 40),
              actions: const <Widget>[
                CupertinoDialogAction(child: Text('OK')),
                CupertinoDialogAction(
                  isDestructiveAction: true,
                  child: Text('This is too long to fit'),
                ),
              ],
              scrollController: scrollController,
            );
          },
        ),
      );

      await tester.tap(find.text('Go'));
      await tester.pumpAndSettle();

      final RenderBox actionsSectionBox = findScrollableActionsSectionRenderBox(tester);

      expect(actionsSectionBox.size.height, 67.8);
    },
  );

  testWidgets(
    'Actions section height for 2 stacked buttons without enough room and large accessibility font is 50% of dialog height.',
    (WidgetTester tester) async {
      final ScrollController scrollController = ScrollController();
      addTearDown(scrollController.dispose);
      await tester.pumpWidget(
        createAppWithButtonThatLaunchesDialog(
          dialogBuilder: (BuildContext context) {
            return MediaQuery.withClampedTextScaling(
              minScaleFactor: 3.0,
              maxScaleFactor: 3.0,
              child: CupertinoAlertDialog(
                title: const Text('The Title'),
                content: Text('The message\n' * 20),
                actions: const <Widget>[
                  CupertinoDialogAction(child: Text('This button is multi line')),
                  CupertinoDialogAction(
                    isDestructiveAction: true,
                    child: Text('This button is multi line'),
                  ),
                ],
                scrollController: scrollController,
              ),
            );
          },
        ),
      );

      await tester.tap(find.text('Go'));
      await tester.pumpAndSettle();

      final RenderBox actionsSectionBox = findScrollableActionsSectionRenderBox(tester);

      // The two multiline buttons with large text are taller than 50% of the
      // dialog height, but with the accessibility layout policy, the 2 buttons
      // should be in a scrollable area equal to half the dialog height.
      expect(actionsSectionBox.size.height, 280.0 - 24.0);
    },
  );

  testWidgets('Actions section height for 3 buttons without enough room is 1.5 buttons tall.', (
    WidgetTester tester,
  ) async {
    final ScrollController scrollController = ScrollController();
    addTearDown(scrollController.dispose);
    await tester.pumpWidget(
      createAppWithButtonThatLaunchesDialog(
        dialogBuilder: (BuildContext context) {
          return CupertinoAlertDialog(
            title: const Text('The Title'),
            content: Text('The message\n' * 40),
            actions: const <Widget>[
              CupertinoDialogAction(child: Text('Option 1')),
              CupertinoDialogAction(child: Text('Option 2')),
              CupertinoDialogAction(child: Text('Option 3')),
            ],
            scrollController: scrollController,
          );
        },
      ),
    );

    await tester.tap(find.text('Go'));
    await tester.pump();
    await tester.pumpAndSettle();

    final RenderBox option1ButtonBox = findActionButtonRenderBoxByTitle(tester, 'Option 1');
    final RenderBox option2ButtonBox = findActionButtonRenderBoxByTitle(tester, 'Option 2');
    final RenderBox actionsSectionBox = findScrollableActionsSectionRenderBox(tester);

    expect(option1ButtonBox.size.width, option2ButtonBox.size.width);
    expect(option1ButtonBox.size.width, actionsSectionBox.size.width);

    // Expected Height = button 1 + divider + 1/2 button 2 = 67.80000000000001
    const double expectedHeight = 67.80000000000001;
    expect(actionsSectionBox.size.height, moreOrLessEquals(expectedHeight));
  });

  testWidgets('Actions section correctly renders overscrolls', (WidgetTester tester) async {
    // Verifies that when the actions section overscrolls, the overscroll part
    // is correctly covered with background.
    final ScrollController actionScrollController = ScrollController();
    addTearDown(actionScrollController.dispose);
    await tester.pumpWidget(
      createAppWithButtonThatLaunchesDialog(
        dialogBuilder: (BuildContext context) {
          return CupertinoAlertDialog(
            actions: List<Widget>.generate(
              12,
              (int i) => CupertinoDialogAction(onPressed: () {}, child: Text('Button ${'*' * i}')),
            ),
          );
        },
      ),
    );

    await tester.tap(find.text('Go'));
    await tester.pumpAndSettle();

    final TestGesture gesture = await tester.startGesture(tester.getCenter(find.text('Button *')));
    await tester.pumpAndSettle();
    // The button should be pressed now, since the scrolling gesture has not
    // taken over.
    await expectLater(
      find.byType(CupertinoAlertDialog),
      matchesGoldenFile('cupertinoAlertDialog.overscroll.0.png'),
    );
    // The dragging gesture must be dispatched in at least two segments.
    // After the first movement, the gesture is started, but the delta is still
    // zero. The second movement gives the delta.
    await gesture.moveBy(const Offset(0, 40));
    await tester.pumpAndSettle();
    await gesture.moveBy(const Offset(0, 100));
    // Test the top overscroll. Use `pump` not `pumpAndSettle` to verify the
    // rendering result of the immediate next frame.
    await tester.pump();
    await expectLater(
      find.byType(CupertinoAlertDialog),
      matchesGoldenFile('cupertinoAlertDialog.overscroll.1.png'),
    );

    await gesture.moveBy(const Offset(0, -300));
    // Test the bottom overscroll. Use `pump` not `pumpAndSettle` to verify the
    // rendering result of the immediate next frame.
    await tester.pump();
    await expectLater(
      find.byType(CupertinoAlertDialog),
      matchesGoldenFile('cupertinoAlertDialog.overscroll.2.png'),
    );
    await gesture.up();
  });

  testWidgets('Actions section correctly renders overscrolls with very far scrolls', (
    WidgetTester tester,
  ) async {
    // When the scroll is really far, the overscroll might be longer than the
    // actions section, causing overflow if not controlled.
    final ScrollController actionScrollController = ScrollController();
    addTearDown(actionScrollController.dispose);
    await tester.pumpWidget(
      createAppWithButtonThatLaunchesDialog(
        dialogBuilder: (BuildContext context) {
          return CupertinoAlertDialog(
            content: Text('content' * 1000),
            actions: List<Widget>.generate(
              4,
              (int i) => CupertinoActionSheetAction(onPressed: () {}, child: Text('Button $i')),
            ),
          );
        },
      ),
    );

    await tester.tap(find.text('Go'));
    await tester.pumpAndSettle();

    final TestGesture gesture = await tester.startGesture(tester.getCenter(find.text('Button 0')));
    await tester.pumpAndSettle();
    await gesture.moveBy(const Offset(0, 40)); // A short drag to start the gesture.
    await tester.pumpAndSettle();
    // The drag is far enough to make the overscroll longer than the section.
    await gesture.moveBy(const Offset(0, 1000));
    await tester.pumpAndSettle();
    // The buttons should be out of the screen
    expect(
      tester.getTopLeft(find.text('Button 0')).dy,
      greaterThan(tester.getBottomLeft(find.byType(ClipRRect)).dy),
    );
    await expectLater(
      find.byType(CupertinoAlertDialog),
      matchesGoldenFile('cupertinoAlertDialog.long-overscroll.0.png'),
    );
  });

  testWidgets('ScaleTransition animation for showCupertinoDialog()', (WidgetTester tester) async {
    await tester.pumpWidget(
      CupertinoApp(
        home: Center(
          child: Builder(
            builder: (BuildContext context) {
              return CupertinoButton(
                onPressed: () {
                  showCupertinoDialog<void>(
                    context: context,
                    builder: (BuildContext context) {
                      return CupertinoAlertDialog(
                        title: const Text('The title'),
                        content: const Text('The content'),
                        actions: <Widget>[
                          const CupertinoDialogAction(child: Text('Cancel')),
                          CupertinoDialogAction(
                            isDestructiveAction: true,
                            onPressed: () {
                              Navigator.pop(context);
                            },
                            child: const Text('Delete'),
                          ),
                        ],
                      );
                    },
                  );
                },
                child: const Text('Go'),
              );
            },
          ),
        ),
      ),
    );

    await tester.tap(find.text('Go'));

    // Enter animation.
    await tester.pump();
    Transform transform = tester.widget(find.byType(Transform));
    expect(transform.transform[0], moreOrLessEquals(1.3, epsilon: 0.001));

    await tester.pump(const Duration(milliseconds: 50));
    transform = tester.widget(find.byType(Transform));
    expect(transform.transform[0], moreOrLessEquals(1.205, epsilon: 0.001));

    await tester.pump(const Duration(milliseconds: 50));
    transform = tester.widget(find.byType(Transform));
    expect(transform.transform[0], moreOrLessEquals(1.100, epsilon: 0.001));

    await tester.pump(const Duration(milliseconds: 50));
    transform = tester.widget(find.byType(Transform));
    expect(transform.transform[0], moreOrLessEquals(1.043, epsilon: 0.001));

    await tester.pump(const Duration(milliseconds: 50));
    transform = tester.widget(find.byType(Transform));
    expect(transform.transform[0], moreOrLessEquals(1.017, epsilon: 0.001));

    await tester.pump(const Duration(milliseconds: 50));
    transform = tester.widget(find.byType(Transform));
    expect(transform.transform[0], moreOrLessEquals(1.006, epsilon: 0.001));

    await tester.pump(const Duration(milliseconds: 50));
    transform = tester.widget(find.byType(Transform));
    expect(transform.transform[0], moreOrLessEquals(1.002, epsilon: 0.001));

    await tester.tap(find.text('Delete'));

    await tester.pump();
    await tester.pump(const Duration(milliseconds: 50));

    // No scaling on exit animation.
    expect(find.byType(Transform), findsNothing);
  });

  testWidgets('FadeTransition animation for showCupertinoDialog()', (WidgetTester tester) async {
    await tester.pumpWidget(
      CupertinoApp(
        home: Center(
          child: Builder(
            builder: (BuildContext context) {
              return CupertinoButton(
                onPressed: () {
                  showCupertinoDialog<void>(
                    context: context,
                    builder: (BuildContext context) {
                      return CupertinoAlertDialog(
                        title: const Text('The title'),
                        content: const Text('The content'),
                        actions: <Widget>[
                          const CupertinoDialogAction(child: Text('Cancel')),
                          CupertinoDialogAction(
                            isDestructiveAction: true,
                            onPressed: () {
                              Navigator.pop(context);
                            },
                            child: const Text('Delete'),
                          ),
                        ],
                      );
                    },
                  );
                },
                child: const Text('Go'),
              );
            },
          ),
        ),
      ),
    );

    await tester.tap(find.text('Go'));

    // Enter animation.
    await tester.pump();
    final Finder fadeTransitionFinder = find.ancestor(
      of: find.byType(CupertinoAlertDialog),
      matching: find.byType(FadeTransition),
    );
    FadeTransition transition = tester.firstWidget(fadeTransitionFinder);

    await tester.pump(const Duration(milliseconds: 50));
    transition = tester.firstWidget(fadeTransitionFinder);
    expect(transition.opacity.value, moreOrLessEquals(0.316, epsilon: 0.001));

    await tester.pump(const Duration(milliseconds: 50));
    transition = tester.firstWidget(fadeTransitionFinder);
    expect(transition.opacity.value, moreOrLessEquals(0.665, epsilon: 0.001));

    await tester.pump(const Duration(milliseconds: 50));
    transition = tester.firstWidget(fadeTransitionFinder);
    expect(transition.opacity.value, moreOrLessEquals(0.856, epsilon: 0.001));

    await tester.pump(const Duration(milliseconds: 50));
    transition = tester.firstWidget(fadeTransitionFinder);
    expect(transition.opacity.value, moreOrLessEquals(0.942, epsilon: 0.001));

    await tester.pump(const Duration(milliseconds: 50));
    transition = tester.firstWidget(fadeTransitionFinder);
    expect(transition.opacity.value, moreOrLessEquals(0.977, epsilon: 0.001));

    await tester.pump(const Duration(milliseconds: 50));
    transition = tester.firstWidget(fadeTransitionFinder);
    expect(transition.opacity.value, moreOrLessEquals(0.991, epsilon: 0.001));

    await tester.pump(const Duration(milliseconds: 50));
    transition = tester.firstWidget(fadeTransitionFinder);
    expect(transition.opacity.value, moreOrLessEquals(0.997, epsilon: 0.001));

    await tester.tap(find.text('Delete'));

    // Exit animation, look at reverse FadeTransition.
    await tester.pump(const Duration(milliseconds: 50));
    transition = tester.firstWidget(fadeTransitionFinder);
    expect(transition.opacity.value, moreOrLessEquals(0.997, epsilon: 0.001));

    await tester.pump(const Duration(milliseconds: 50));
    transition = tester.firstWidget(fadeTransitionFinder);
    expect(transition.opacity.value, moreOrLessEquals(0.681, epsilon: 0.001));

    await tester.pump(const Duration(milliseconds: 50));
    transition = tester.firstWidget(fadeTransitionFinder);
    expect(transition.opacity.value, moreOrLessEquals(0.333, epsilon: 0.001));

    await tester.pump(const Duration(milliseconds: 50));
    transition = tester.firstWidget(fadeTransitionFinder);
    expect(transition.opacity.value, moreOrLessEquals(0.143, epsilon: 0.001));

    await tester.pump(const Duration(milliseconds: 50));
    transition = tester.firstWidget(fadeTransitionFinder);
    expect(transition.opacity.value, moreOrLessEquals(0.057, epsilon: 0.001));

    await tester.pump(const Duration(milliseconds: 50));
    transition = tester.firstWidget(fadeTransitionFinder);
    expect(transition.opacity.value, moreOrLessEquals(0.022, epsilon: 0.001));
  });

  testWidgets('Actions are accessible by key', (WidgetTester tester) async {
    await tester.pumpWidget(
      createAppWithButtonThatLaunchesDialog(
        dialogBuilder: (BuildContext context) {
          return const CupertinoAlertDialog(
            title: Text('The Title'),
            content: Text('The message'),
            actions: <Widget>[
              CupertinoDialogAction(key: Key('option_1'), child: Text('Option 1')),
              CupertinoDialogAction(key: Key('option_2'), child: Text('Option 2')),
            ],
          );
        },
      ),
    );

    await tester.tap(find.text('Go'));
    await tester.pump();

    expect(find.byKey(const Key('option_1')), findsOneWidget);
    expect(find.byKey(const Key('option_2')), findsOneWidget);
    expect(find.byKey(const Key('option_3')), findsNothing);
  });

  testWidgets('Dialog widget insets by MediaQuery viewInsets', (WidgetTester tester) async {
    await tester.pumpWidget(
      const CupertinoApp(
        home: MediaQuery(
          data: MediaQueryData(),
          child: CupertinoAlertDialog(content: Placeholder(fallbackHeight: 200.0)),
        ),
      ),
    );

    final Rect placeholderRectWithoutInsets = tester.getRect(find.byType(Placeholder));

    await tester.pumpWidget(
      const CupertinoApp(
        home: MediaQuery(
          data: MediaQueryData(viewInsets: EdgeInsets.fromLTRB(40.0, 30.0, 20.0, 10.0)),
          child: CupertinoAlertDialog(content: Placeholder(fallbackHeight: 200.0)),
        ),
      ),
    );

    // no change yet because padding is animated
    expect(tester.getRect(find.byType(Placeholder)), placeholderRectWithoutInsets);

    await tester.pump(const Duration(seconds: 1));

    // once animation settles the dialog is padded by the new viewInsets
    expect(
      tester.getRect(find.byType(Placeholder)),
      placeholderRectWithoutInsets.translate(10, 10),
    );
  });

  testWidgets('showCupertinoDialog - custom barrierLabel', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);

    await tester.pumpWidget(
      CupertinoApp(
        home: Builder(
          builder: (BuildContext context) {
            return Center(
              child: CupertinoButton(
                child: const Text('X'),
                onPressed: () {
                  showCupertinoDialog<void>(
                    context: context,
                    barrierLabel: 'Custom label',
                    builder: (BuildContext context) {
                      return const CupertinoAlertDialog(
                        title: Text('Title'),
                        content: Text('Content'),
                        actions: <Widget>[
                          CupertinoDialogAction(child: Text('Yes')),
                          CupertinoDialogAction(child: Text('No')),
                        ],
                      );
                    },
                  );
                },
              ),
            );
          },
        ),
      ),
    );

    expect(
      semantics,
      isNot(
        includesNodeWith(label: 'Custom label', flags: <SemanticsFlag>[SemanticsFlag.namesRoute]),
      ),
    );
    semantics.dispose();
  });

  testWidgets('CupertinoDialogRoute is state restorable', (WidgetTester tester) async {
    await tester.pumpWidget(
      const CupertinoApp(restorationScopeId: 'app', home: _RestorableDialogTestWidget()),
    );

    expect(find.byType(CupertinoAlertDialog), findsNothing);

    await tester.tap(find.text('X'));
    await tester.pumpAndSettle();

    expect(find.byType(CupertinoAlertDialog), findsOneWidget);
    final TestRestorationData restorationData = await tester.getRestorationData();

    await tester.restartAndRestore();

    expect(find.byType(CupertinoAlertDialog), findsOneWidget);

    // Tap on the barrier.
    await tester.tapAt(const Offset(10.0, 10.0));
    await tester.pumpAndSettle();

    expect(find.byType(CupertinoAlertDialog), findsNothing);

    await tester.restoreFrom(restorationData);
    expect(find.byType(CupertinoAlertDialog), findsOneWidget);
  }, skip: isBrowser); // https://github.com/flutter/flutter/issues/33615

  testWidgets(
    'Conflicting scrollbars are not applied by ScrollBehavior to CupertinoAlertDialog',
    (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/issues/83819
      final ScrollController actionScrollController = ScrollController();
      addTearDown(actionScrollController.dispose);
      await tester.pumpWidget(
        createAppWithButtonThatLaunchesDialog(
          dialogBuilder: (BuildContext context) {
            return MediaQuery.withNoTextScaling(
              child: CupertinoAlertDialog(
                title: const Text('Test Title'),
                content: const Text('Test Content'),
                actions: const <Widget>[
                  CupertinoDialogAction(child: Text('One')),
                  CupertinoDialogAction(child: Text('Two')),
                ],
                actionScrollController: actionScrollController,
              ),
            );
          },
        ),
      );

      await tester.tap(find.text('Go'));
      await tester.pump();

      // The inherited ScrollBehavior should not apply Scrollbars since they are
      // already built in to the widget.
      expect(find.byType(Scrollbar), findsNothing);
      expect(find.byType(RawScrollbar), findsNothing);
      // Built in CupertinoScrollbars should only number 2: one for the actions,
      // one for the content.
      expect(find.byType(CupertinoScrollbar), findsNWidgets(2));
    },
    variant: TargetPlatformVariant.all(),
  );

  testWidgets('CupertinoAlertDialog scrollbars controllers should be different', (
    WidgetTester tester,
  ) async {
    // https://github.com/flutter/flutter/pull/81278
    await tester.pumpWidget(
      const CupertinoApp(
        home: MediaQuery(
          data: MediaQueryData(),
          child: CupertinoAlertDialog(
            actions: <Widget>[CupertinoDialogAction(child: Text('OK'))],
            content: Placeholder(fallbackHeight: 200.0),
          ),
        ),
      ),
    );

    final List<CupertinoScrollbar> scrollbars =
        find
            .descendant(
              of: find.byType(CupertinoAlertDialog),
              matching: find.byType(CupertinoScrollbar),
            )
            .evaluate()
            .map((Element e) => e.widget as CupertinoScrollbar)
            .toList();

    expect(scrollbars.length, 2);
    expect(scrollbars[0].controller != scrollbars[1].controller, isTrue);
  });

  group('showCupertinoDialog avoids overlapping display features', () {
    testWidgets('positioning using anchorPoint', (WidgetTester tester) async {
      await tester.pumpWidget(
        CupertinoApp(
          builder: (BuildContext context, Widget? child) {
            return MediaQuery(
              // Display has a vertical hinge down the middle
              data: const MediaQueryData(
                size: Size(800, 600),
                displayFeatures: <DisplayFeature>[
                  DisplayFeature(
                    bounds: Rect.fromLTRB(390, 0, 410, 600),
                    type: DisplayFeatureType.hinge,
                    state: DisplayFeatureState.unknown,
                  ),
                ],
              ),
              child: child!,
            );
          },
          home: const Center(child: Text('Test')),
        ),
      );

      final BuildContext context = tester.element(find.text('Test'));
      showCupertinoDialog<void>(
        context: context,
        builder: (BuildContext context) {
          return const Placeholder();
        },
        anchorPoint: const Offset(1000, 0),
      );
      await tester.pumpAndSettle();

      // Should take the right side of the screen
      expect(tester.getTopLeft(find.byType(Placeholder)), const Offset(410.0, 0.00));
      expect(tester.getBottomRight(find.byType(Placeholder)), const Offset(800.0, 600.0));
    });

    testWidgets('positioning using Directionality', (WidgetTester tester) async {
      await tester.pumpWidget(
        CupertinoApp(
          builder: (BuildContext context, Widget? child) {
            return MediaQuery(
              // Display has a vertical hinge down the middle
              data: const MediaQueryData(
                size: Size(800, 600),
                displayFeatures: <DisplayFeature>[
                  DisplayFeature(
                    bounds: Rect.fromLTRB(390, 0, 410, 600),
                    type: DisplayFeatureType.hinge,
                    state: DisplayFeatureState.unknown,
                  ),
                ],
              ),
              child: Directionality(textDirection: TextDirection.rtl, child: child!),
            );
          },
          home: const Center(child: Text('Test')),
        ),
      );

      final BuildContext context = tester.element(find.text('Test'));
      showCupertinoDialog<void>(
        context: context,
        builder: (BuildContext context) {
          return const Placeholder();
        },
      );
      await tester.pumpAndSettle();

      // Should take the right side of the screen
      expect(tester.getTopLeft(find.byType(Placeholder)), const Offset(410.0, 0.0));
      expect(tester.getBottomRight(find.byType(Placeholder)), const Offset(800.0, 600.0));
    });

    testWidgets('default positioning', (WidgetTester tester) async {
      await tester.pumpWidget(
        CupertinoApp(
          builder: (BuildContext context, Widget? child) {
            return MediaQuery(
              // Display has a vertical hinge down the middle
              data: const MediaQueryData(
                size: Size(800, 600),
                displayFeatures: <DisplayFeature>[
                  DisplayFeature(
                    bounds: Rect.fromLTRB(390, 0, 410, 600),
                    type: DisplayFeatureType.hinge,
                    state: DisplayFeatureState.unknown,
                  ),
                ],
              ),
              child: child!,
            );
          },
          home: const Center(child: Text('Test')),
        ),
      );

      final BuildContext context = tester.element(find.text('Test'));
      showCupertinoDialog<void>(
        context: context,
        builder: (BuildContext context) {
          return const Placeholder();
        },
      );
      await tester.pumpAndSettle();

      // By default it should place the dialog on the left screen
      expect(tester.getTopLeft(find.byType(Placeholder)), Offset.zero);
      expect(tester.getBottomRight(find.byType(Placeholder)), const Offset(390.0, 600.0));
    });
  });

  testWidgets('Hovering over Cupertino alert dialog action updates cursor to clickable on Web', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      createAppWithButtonThatLaunchesDialog(
        dialogBuilder: (BuildContext context) {
          return MediaQuery.withClampedTextScaling(
            minScaleFactor: 3.0,
            maxScaleFactor: 3.0,
            child: RepaintBoundary(
              child: CupertinoAlertDialog(
                title: const Text('Title'),
                content: const Text('text'),
                actions: <Widget>[
                  CupertinoDialogAction(onPressed: () {}, child: const Text('NO')),
                  CupertinoDialogAction(onPressed: () {}, child: const Text('OK')),
                ],
              ),
            ),
          );
        },
      ),
    );

    await tester.tap(find.text('Go'));
    await tester.pumpAndSettle();

    final TestGesture gesture = await tester.createGesture(
      kind: PointerDeviceKind.mouse,
      pointer: 1,
    );
    await gesture.addPointer(location: const Offset(10, 10));
    await tester.pumpAndSettle();
    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.basic,
    );

    final Offset dialogAction = tester.getCenter(find.text('OK'));
    await gesture.moveTo(dialogAction);
    await tester.pumpAndSettle();
    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      kIsWeb ? SystemMouseCursors.click : SystemMouseCursors.basic,
    );
  });
}

RenderBox findActionButtonRenderBoxByTitle(WidgetTester tester, String title) {
  final RenderObject buttonBox = tester.renderObject(
    find.widgetWithText(CupertinoDialogAction, title),
  );
  assert(buttonBox is RenderBox);
  return buttonBox as RenderBox;
}

RenderBox findScrollableActionsSectionRenderBox(WidgetTester tester) {
  final RenderObject actionsSection = tester.renderObject(
    find.byElementPredicate((Element element) {
      return element.widget.runtimeType.toString() == '_CupertinoAlertActionSection';
    }),
  );
  assert(actionsSection is RenderBox);
  return actionsSection as RenderBox;
}

Widget createAppWithButtonThatLaunchesDialog({required WidgetBuilder dialogBuilder}) {
  return CupertinoApp(
    home: Center(
      child: Builder(
        builder: (BuildContext context) {
          return CupertinoButton(
            onPressed: () {
              showCupertinoDialog<void>(context: context, builder: dialogBuilder);
            },
            child: const Text('Go'),
          );
        },
      ),
    ),
  );
}

Widget boilerplate(Widget child) {
  return Directionality(textDirection: TextDirection.ltr, child: child);
}

Widget createAppWithCenteredButton(Widget child) {
  return CupertinoApp(home: Center(child: CupertinoButton(onPressed: null, child: child)));
}

@pragma('vm:entry-point')
class _RestorableDialogTestWidget extends StatelessWidget {
  const _RestorableDialogTestWidget();

  @pragma('vm:entry-point')
  static Route<Object?> _dialogBuilder(BuildContext context, Object? arguments) {
    return CupertinoDialogRoute<void>(
      context: context,
      builder: (BuildContext context) {
        return const CupertinoAlertDialog(
          title: Text('Title'),
          content: Text('Content'),
          actions: <Widget>[
            CupertinoDialogAction(child: Text('Yes')),
            CupertinoDialogAction(child: Text('No')),
          ],
        );
      },
    );
  }

  @override
  Widget build(BuildContext context) {
    return CupertinoPageScaffold(
      navigationBar: const CupertinoNavigationBar(middle: Text('Home')),
      child: Center(
        child: CupertinoButton(
          onPressed: () {
            Navigator.of(context).restorablePush(_dialogBuilder);
          },
          child: const Text('X'),
        ),
      ),
    );
  }
}

// Shows an app that has a button with text "Go", and clicking this button
// displays the `dialog` and hides the button.
//
// The `theme` will be applied to the app and determines the background.
class TestScaffoldApp extends StatefulWidget {
  const TestScaffoldApp({super.key, required this.theme, required this.dialog});
  final CupertinoThemeData theme;
  final Widget dialog;

  @override
  TestScaffoldAppState createState() => TestScaffoldAppState();
}

class TestScaffoldAppState extends State<TestScaffoldApp> {
  bool _pressedButton = false;

  @override
  Widget build(BuildContext context) {
    return CupertinoApp(
      // Hide the debug banner. Because this CupertinoApp is captured in golden
      // test as a whole. The debug banner contains tilted text, whose
      // anti-alias might cause false negative result.
      // https://github.com/flutter/flutter/pull/150442
      debugShowCheckedModeBanner: false,
      theme: widget.theme,
      home: Builder(
        builder:
            (BuildContext context) => CupertinoPageScaffold(
              child: Center(
                child:
                    _pressedButton
                        ? Container()
                        : CupertinoButton(
                          onPressed: () {
                            setState(() {
                              _pressedButton = true;
                            });
                            showCupertinoDialog<void>(
                              context: context,
                              builder: (BuildContext context) {
                                return widget.dialog;
                              },
                            );
                          },
                          child: const Text('Go'),
                        ),
              ),
            ),
      ),
    );
  }
}

// Old-style action sheet buttons, which are implemented with
// `GestureDetector.onTap`.
class LegacyAction extends StatelessWidget {
  const LegacyAction({super.key, required this.onPressed, required this.child});

  final VoidCallback onPressed;
  final Widget child;

  @override
  Widget build(BuildContext context) {
    return GestureDetector(
      onTap: onPressed,
      behavior: HitTestBehavior.opaque,
      child: ConstrainedBox(
        constraints: const BoxConstraints(minHeight: 45),
        child: Container(
          alignment: AlignmentDirectional.center,
          padding: const EdgeInsets.symmetric(vertical: 16.0, horizontal: 10.0),
          child: child,
        ),
      ),
    );
  }
}
