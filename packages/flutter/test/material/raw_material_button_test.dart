// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/src/services/keyboard_key.g.dart';
import 'package:flutter_test/flutter_test.dart';
import '../widgets/semantics_tester.dart';

void main() {
  testWidgets('RawMaterialButton responds when tapped', (WidgetTester tester) async {
    bool pressed = false;
    const Color splashColor = Color(0xff00ff00);
    await tester.pumpWidget(
      Theme(
        data: ThemeData(useMaterial3: false),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: Center(
            child: RawMaterialButton(
              splashColor: splashColor,
              onPressed: () {
                pressed = true;
              },
              child: const Text('BUTTON'),
            ),
          ),
        ),
      ),
    );

    await tester.tap(find.text('BUTTON'));
    await tester.pump(const Duration(milliseconds: 10));

    final RenderBox splash = Material.of(tester.element(find.byType(InkWell))) as RenderBox;
    expect(splash, paints..circle(color: splashColor));

    await tester.pumpAndSettle();

    expect(pressed, isTrue);
  });

  testWidgets('RawMaterialButton responds to shortcut when activated', (WidgetTester tester) async {
    bool pressed = false;
    final FocusNode focusNode = FocusNode(debugLabel: 'Test Button');
    const Color splashColor = Color(0xff00ff00);
    await tester.pumpWidget(
      Theme(
        data: ThemeData(useMaterial3: false),
        child: Shortcuts(
          shortcuts: const <ShortcutActivator, Intent>{
            SingleActivator(LogicalKeyboardKey.enter): ActivateIntent(),
            SingleActivator(LogicalKeyboardKey.space): ActivateIntent(),
          },
          child: Directionality(
            textDirection: TextDirection.ltr,
            child: Center(
              child: RawMaterialButton(
                splashColor: splashColor,
                focusNode: focusNode,
                onPressed: () {
                  pressed = true;
                },
                child: const Text('BUTTON'),
              ),
            ),
          ),
        ),
      ),
    );

    focusNode.requestFocus();
    await tester.pump();

    // Web doesn't react to enter, just space.
    await tester.sendKeyEvent(LogicalKeyboardKey.enter);
    await tester.pump(const Duration(milliseconds: 10));

    if (!kIsWeb) {
      final RenderBox splash = Material.of(tester.element(find.byType(InkWell))) as RenderBox;
      expect(splash, paints..circle(color: splashColor));
    }

    await tester.pumpAndSettle();

    expect(pressed, isTrue);

    pressed = false;
    await tester.sendKeyEvent(LogicalKeyboardKey.space);
    await tester.pumpAndSettle();

    expect(pressed, isTrue);

    pressed = false;
    await tester.sendKeyEvent(LogicalKeyboardKey.space);
    await tester.pump(const Duration(milliseconds: 10));

    final RenderBox splash = Material.of(tester.element(find.byType(InkWell))) as RenderBox;
    expect(splash, paints..circle(color: splashColor));

    await tester.pumpAndSettle();

    expect(pressed, isTrue);

    pressed = false;
    await tester.sendKeyEvent(LogicalKeyboardKey.space);
    await tester.pumpAndSettle();

    expect(pressed, isTrue);
    focusNode.dispose();
  });

  testWidgets('materialTapTargetSize.padded expands hit test area', (WidgetTester tester) async {
    int pressed = 0;

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: RawMaterialButton(
          onPressed: () {
            pressed++;
          },
          constraints: BoxConstraints.tight(const Size(10.0, 10.0)),
          materialTapTargetSize: MaterialTapTargetSize.padded,
          child: const Text('+'),
        ),
      ),
    );

    await tester.tapAt(const Offset(40.0, 400.0));

    expect(pressed, 1);
  });

  testWidgets('materialTapTargetSize.padded expands semantics area', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Center(
          child: RawMaterialButton(
            onPressed: () {},
            constraints: BoxConstraints.tight(const Size(10.0, 10.0)),
            materialTapTargetSize: MaterialTapTargetSize.padded,
            child: const Text('+'),
          ),
        ),
      ),
    );

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics(
              id: 1,
              flags: <SemanticsFlag>[
                SemanticsFlag.hasEnabledState,
                SemanticsFlag.isButton,
                SemanticsFlag.isEnabled,
                SemanticsFlag.isFocusable,
              ],
              actions: <SemanticsAction>[SemanticsAction.tap, SemanticsAction.focus],
              label: '+',
              textDirection: TextDirection.ltr,
              rect: const Rect.fromLTRB(0.0, 0.0, 48.0, 48.0),
              children: <TestSemantics>[],
            ),
          ],
        ),
        ignoreTransform: true,
      ),
    );

    semantics.dispose();
  });

  testWidgets('Ink splash from center tap originates in correct location', (
    WidgetTester tester,
  ) async {
    const Color highlightColor = Color(0xAAFF0000);
    const Color splashColor = Color(0xAA0000FF);
    const Color fillColor = Color(0xFFEF5350);

    await tester.pumpWidget(
      Theme(
        data: ThemeData(useMaterial3: false),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: Center(
            child: RawMaterialButton(
              materialTapTargetSize: MaterialTapTargetSize.padded,
              onPressed: () {},
              fillColor: fillColor,
              highlightColor: highlightColor,
              splashColor: splashColor,
              child: const SizedBox(),
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
    // centered in material button.
    expect(box, paints..circle(x: 44.0, y: 18.0, color: splashColor));
    await gesture.up();
  });

  testWidgets('Ink splash from tap above material originates in correct location', (
    WidgetTester tester,
  ) async {
    const Color highlightColor = Color(0xAAFF0000);
    const Color splashColor = Color(0xAA0000FF);
    const Color fillColor = Color(0xFFEF5350);

    await tester.pumpWidget(
      Theme(
        data: ThemeData(useMaterial3: false),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: Center(
            child: RawMaterialButton(
              materialTapTargetSize: MaterialTapTargetSize.padded,
              onPressed: () {},
              fillColor: fillColor,
              highlightColor: highlightColor,
              splashColor: splashColor,
              child: const SizedBox(),
            ),
          ),
        ),
      ),
    );

    final Offset top = tester.getRect(find.byType(InkWell)).topCenter;
    final TestGesture gesture = await tester.startGesture(top);
    await tester.pump(); // start gesture
    await tester.pump(const Duration(milliseconds: 200)); // wait for splash to be well under way
    final RenderBox box = Material.of(tester.element(find.byType(InkWell))) as RenderBox;
    // paints above material
    expect(box, paints..circle(x: 44.0, y: 0.0, color: splashColor));
    await gesture.up();
  });

  testWidgets('off-center child is hit testable', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Column(
          children: <Widget>[
            RawMaterialButton(
              materialTapTargetSize: MaterialTapTargetSize.padded,
              onPressed: () {},
              child: const SizedBox(
                width: 400.0,
                height: 400.0,
                child: Column(
                  mainAxisAlignment: MainAxisAlignment.end,
                  children: <Widget>[SizedBox(height: 50.0, width: 400.0, child: Text('Material'))],
                ),
              ),
            ),
          ],
        ),
      ),
    );
    expect(find.text('Material').hitTestable(), findsOneWidget);
  });

  testWidgets('smaller child is hit testable', (WidgetTester tester) async {
    const Key key = Key('test');
    await tester.pumpWidget(
      MaterialApp(
        home: Column(
          children: <Widget>[
            RawMaterialButton(
              materialTapTargetSize: MaterialTapTargetSize.padded,
              onPressed: () {},
              child: SizedBox(
                key: key,
                width: 8.0,
                height: 8.0,
                child: Container(color: const Color(0xFFAABBCC)),
              ),
            ),
          ],
        ),
      ),
    );
    expect(find.byKey(key).hitTestable(), findsOneWidget);
  });

  testWidgets('RawMaterialButton can be expanded by parent constraints', (
    WidgetTester tester,
  ) async {
    const Key key = Key('test');
    await tester.pumpWidget(
      MaterialApp(
        home: Column(
          crossAxisAlignment: CrossAxisAlignment.stretch,
          children: <Widget>[
            RawMaterialButton(key: key, onPressed: () {}, child: const SizedBox()),
          ],
        ),
      ),
    );

    expect(tester.getSize(find.byKey(key)), const Size(800.0, 48.0));
  });

  testWidgets('RawMaterialButton handles focus', (WidgetTester tester) async {
    final FocusNode focusNode = FocusNode(debugLabel: 'Button Focus');
    const Key key = Key('test');
    const Color focusColor = Color(0xff00ff00);

    FocusManager.instance.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    await tester.pumpWidget(
      MaterialApp(
        home: Center(
          child: RawMaterialButton(
            key: key,
            focusNode: focusNode,
            focusColor: focusColor,
            onPressed: () {},
            child: Container(width: 100, height: 100, color: const Color(0xffff0000)),
          ),
        ),
      ),
    );
    final RenderBox box = Material.of(tester.element(find.byType(InkWell))) as RenderBox;
    expect(box, isNot(paints..rect(color: focusColor)));

    focusNode.requestFocus();
    await tester.pumpAndSettle(const Duration(seconds: 1));

    expect(box, paints..rect(color: focusColor));
    focusNode.dispose();
  });

  testWidgets('RawMaterialButton loses focus when disabled.', (WidgetTester tester) async {
    final FocusNode focusNode = FocusNode(debugLabel: 'RawMaterialButton');
    await tester.pumpWidget(
      MaterialApp(
        home: Center(
          child: RawMaterialButton(
            autofocus: true,
            focusNode: focusNode,
            onPressed: () {},
            child: Container(width: 100, height: 100, color: const Color(0xffff0000)),
          ),
        ),
      ),
    );

    await tester.pump();
    expect(focusNode.hasPrimaryFocus, isTrue);

    await tester.pumpWidget(
      MaterialApp(
        home: Center(
          child: RawMaterialButton(
            focusNode: focusNode,
            onPressed: null,
            child: Container(width: 100, height: 100, color: const Color(0xffff0000)),
          ),
        ),
      ),
    );

    await tester.pump();
    expect(focusNode.hasPrimaryFocus, isFalse);
    focusNode.dispose();
  });

  testWidgets("Disabled RawMaterialButton can't be traversed to.", (WidgetTester tester) async {
    final FocusNode focusNode1 = FocusNode(debugLabel: '$RawMaterialButton 1');
    final FocusNode focusNode2 = FocusNode(debugLabel: '$RawMaterialButton 2');

    await tester.pumpWidget(
      MaterialApp(
        home: FocusScope(
          child: Center(
            child: Column(
              children: <Widget>[
                RawMaterialButton(
                  autofocus: true,
                  focusNode: focusNode1,
                  onPressed: () {},
                  child: Container(width: 100, height: 100, color: const Color(0xffff0000)),
                ),
                RawMaterialButton(
                  autofocus: true,
                  focusNode: focusNode2,
                  onPressed: null,
                  child: Container(width: 100, height: 100, color: const Color(0xffff0000)),
                ),
              ],
            ),
          ),
        ),
      ),
    );
    await tester.pump();

    expect(focusNode1.hasPrimaryFocus, isTrue);
    expect(focusNode2.hasPrimaryFocus, isFalse);

    expect(focusNode1.nextFocus(), isFalse);
    await tester.pump();

    expect(focusNode1.hasPrimaryFocus, isTrue);
    expect(focusNode2.hasPrimaryFocus, isFalse);

    focusNode1.dispose();
    focusNode2.dispose();
  });

  testWidgets('RawMaterialButton handles hover', (WidgetTester tester) async {
    const Key key = Key('test');
    const Color hoverColor = Color(0xff00ff00);

    FocusManager.instance.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    await tester.pumpWidget(
      MaterialApp(
        home: Center(
          child: RawMaterialButton(
            key: key,
            hoverColor: hoverColor,
            hoverElevation: 10.5,
            onPressed: () {},
            child: Container(width: 100, height: 100, color: const Color(0xffff0000)),
          ),
        ),
      ),
    );
    final RenderBox box = Material.of(tester.element(find.byType(InkWell))) as RenderBox;
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    expect(box, isNot(paints..rect(color: hoverColor)));

    await gesture.moveTo(tester.getCenter(find.byType(RawMaterialButton)));
    await tester.pumpAndSettle(const Duration(seconds: 1));

    expect(box, paints..rect(color: hoverColor));
  });

  testWidgets(
    'RawMaterialButton onPressed and onLongPress callbacks are correctly called when non-null',
    (WidgetTester tester) async {
      bool wasPressed;
      Finder rawMaterialButton;

      Widget buildFrame({VoidCallback? onPressed, VoidCallback? onLongPress}) {
        return Directionality(
          textDirection: TextDirection.ltr,
          child: RawMaterialButton(
            onPressed: onPressed,
            onLongPress: onLongPress,
            child: const Text('button'),
          ),
        );
      }

      // onPressed not null, onLongPress null.
      wasPressed = false;
      await tester.pumpWidget(
        buildFrame(
          onPressed: () {
            wasPressed = true;
          },
        ),
      );
      rawMaterialButton = find.byType(RawMaterialButton);
      expect(tester.widget<RawMaterialButton>(rawMaterialButton).enabled, true);
      await tester.tap(rawMaterialButton);
      expect(wasPressed, true);

      // onPressed null, onLongPress not null.
      wasPressed = false;
      await tester.pumpWidget(
        buildFrame(
          onLongPress: () {
            wasPressed = true;
          },
        ),
      );
      rawMaterialButton = find.byType(RawMaterialButton);
      expect(tester.widget<RawMaterialButton>(rawMaterialButton).enabled, true);
      await tester.longPress(rawMaterialButton);
      expect(wasPressed, true);

      // onPressed null, onLongPress null.
      await tester.pumpWidget(buildFrame());
      rawMaterialButton = find.byType(RawMaterialButton);
      expect(tester.widget<RawMaterialButton>(rawMaterialButton).enabled, false);
    },
  );

  testWidgets('RawMaterialButton onPressed and onLongPress callbacks are distinctly recognized', (
    WidgetTester tester,
  ) async {
    bool didPressButton = false;
    bool didLongPressButton = false;

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: RawMaterialButton(
          onPressed: () {
            didPressButton = true;
          },
          onLongPress: () {
            didLongPressButton = true;
          },
          child: const Text('button'),
        ),
      ),
    );

    final Finder rawMaterialButton = find.byType(RawMaterialButton);
    expect(tester.widget<RawMaterialButton>(rawMaterialButton).enabled, true);

    expect(didPressButton, isFalse);
    await tester.tap(rawMaterialButton);
    expect(didPressButton, isTrue);

    expect(didLongPressButton, isFalse);
    await tester.longPress(rawMaterialButton);
    expect(didLongPressButton, isTrue);
  });

  testWidgets('RawMaterialButton responds to density changes.', (WidgetTester tester) async {
    const Key key = Key('test');
    const Key childKey = Key('test child');

    Future<void> buildTest(VisualDensity visualDensity, {bool useText = false}) async {
      return tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(useMaterial3: false),
          home: Directionality(
            textDirection: TextDirection.rtl,
            child: Center(
              child: RawMaterialButton(
                visualDensity: visualDensity,
                key: key,
                onPressed: () {},
                child:
                    useText
                        ? const Text('Text', key: childKey)
                        : Container(
                          key: childKey,
                          width: 100,
                          height: 100,
                          color: const Color(0xffff0000),
                        ),
              ),
            ),
          ),
        ),
      );
    }

    await buildTest(VisualDensity.standard);
    final RenderBox box = tester.renderObject(find.byKey(key));
    Rect childRect = tester.getRect(find.byKey(childKey));
    await tester.pumpAndSettle();
    expect(box.size, equals(const Size(100, 100)));
    expect(childRect, equals(const Rect.fromLTRB(350, 250, 450, 350)));

    await buildTest(const VisualDensity(horizontal: 3.0, vertical: 3.0));
    await tester.pumpAndSettle();
    childRect = tester.getRect(find.byKey(childKey));
    expect(box.size, equals(const Size(124, 124)));
    expect(childRect, equals(const Rect.fromLTRB(350, 250, 450, 350)));

    await buildTest(const VisualDensity(horizontal: -3.0, vertical: -3.0));
    await tester.pumpAndSettle();
    childRect = tester.getRect(find.byKey(childKey));
    expect(box.size, equals(const Size(100, 100)));
    expect(childRect, equals(const Rect.fromLTRB(350, 250, 450, 350)));

    await buildTest(VisualDensity.standard, useText: true);
    await tester.pumpAndSettle();
    childRect = tester.getRect(find.byKey(childKey));
    expect(box.size, equals(const Size(88, 48)));
    expect(childRect, equals(const Rect.fromLTRB(372.0, 293.0, 428.0, 307.0)));

    await buildTest(const VisualDensity(horizontal: 3.0, vertical: 3.0), useText: true);
    await tester.pumpAndSettle();
    childRect = tester.getRect(find.byKey(childKey));
    expect(box.size, equals(const Size(100, 60)));
    expect(childRect, equals(const Rect.fromLTRB(372.0, 293.0, 428.0, 307.0)));

    await buildTest(const VisualDensity(horizontal: -3.0, vertical: -3.0), useText: true);
    await tester.pumpAndSettle();
    childRect = tester.getRect(find.byKey(childKey));
    expect(box.size, equals(const Size(76, 36)));
    expect(childRect, equals(const Rect.fromLTRB(372.0, 293.0, 428.0, 307.0)));
  });

  testWidgets('RawMaterialButton changes mouse cursor when hovered', (WidgetTester tester) async {
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: MouseRegion(
          cursor: SystemMouseCursors.forbidden,
          child: RawMaterialButton(onPressed: () {}, mouseCursor: SystemMouseCursors.text),
        ),
      ),
    );

    final TestGesture gesture = await tester.createGesture(
      kind: PointerDeviceKind.mouse,
      pointer: 1,
    );
    await gesture.addPointer(location: Offset.zero);

    await tester.pump();

    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.text,
    );

    // Test default cursor
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: MouseRegion(
          cursor: SystemMouseCursors.forbidden,
          child: RawMaterialButton(onPressed: () {}),
        ),
      ),
    );

    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.click,
    );

    // Test default cursor when disabled
    await tester.pumpWidget(
      const Directionality(
        textDirection: TextDirection.ltr,
        child: MouseRegion(
          cursor: SystemMouseCursors.forbidden,
          child: RawMaterialButton(onPressed: null),
        ),
      ),
    );

    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.basic,
    );
  });
}
