// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

final LogicalKeyboardKey modifierKey =
    defaultTargetPlatform == TargetPlatform.macOS
        ? LogicalKeyboardKey.metaLeft
        : LogicalKeyboardKey.controlLeft;

class _NoNotificationContextScrollable extends Scrollable {
  const _NoNotificationContextScrollable({super.controller, required super.viewportBuilder});

  @override
  ScrollableState createState() => _NoNotificationContextScrollableState();
}

class _NoNotificationContextScrollableState extends ScrollableState {
  @override
  BuildContext? get notificationContext => null;
}

void main() {
  group('ScrollableDetails', () {
    test('copyWith / == / hashCode', () {
      final ScrollController controller = ScrollController();
      addTearDown(controller.dispose);
      final ScrollableDetails details = ScrollableDetails(
        direction: AxisDirection.down,
        controller: controller,
        physics: const AlwaysScrollableScrollPhysics(),
        decorationClipBehavior: Clip.hardEdge,
      );
      ScrollableDetails copiedDetails = details.copyWith();
      expect(details, copiedDetails);
      expect(details.hashCode, copiedDetails.hashCode);

      copiedDetails = details.copyWith(
        direction: AxisDirection.left,
        physics: const ClampingScrollPhysics(),
        decorationClipBehavior: Clip.none,
      );
      expect(
        copiedDetails,
        ScrollableDetails(
          direction: AxisDirection.left,
          controller: controller,
          physics: const ClampingScrollPhysics(),
          decorationClipBehavior: Clip.none,
        ),
      );
    });

    test('toString', () {
      final ScrollController controller = ScrollController();
      addTearDown(controller.dispose);
      const ScrollableDetails bareDetails = ScrollableDetails(direction: AxisDirection.right);
      expect(
        bareDetails.toString(),
        equalsIgnoringHashCodes('ScrollableDetails#00000(axisDirection: AxisDirection.right)'),
      );
      final ScrollableDetails fullDetails = ScrollableDetails(
        direction: AxisDirection.down,
        controller: controller,
        physics: const AlwaysScrollableScrollPhysics(),
        decorationClipBehavior: Clip.hardEdge,
      );
      expect(
        fullDetails.toString(),
        equalsIgnoringHashCodes(
          'ScrollableDetails#00000('
          'axisDirection: AxisDirection.down, '
          'scroll controller: ScrollController#00000(no clients), '
          'scroll physics: AlwaysScrollableScrollPhysics, '
          'decorationClipBehavior: Clip.hardEdge)',
        ),
      );
    });

    test('deprecated clipBehavior is backwards compatible', () {
      const ScrollableDetails deprecatedClip = ScrollableDetails(
        direction: AxisDirection.right,
        clipBehavior: Clip.hardEdge,
      );
      expect(deprecatedClip.clipBehavior, Clip.hardEdge);
      expect(deprecatedClip.decorationClipBehavior, Clip.hardEdge);

      const ScrollableDetails newClip = ScrollableDetails(
        direction: AxisDirection.right,
        decorationClipBehavior: Clip.hardEdge,
      );
      expect(newClip.clipBehavior, Clip.hardEdge);
      expect(newClip.decorationClipBehavior, Clip.hardEdge);
    });
  });

  testWidgets(
    "Keyboard scrolling doesn't happen if scroll physics are set to NeverScrollableScrollPhysics",
    (WidgetTester tester) async {
      final ScrollController controller = ScrollController();
      addTearDown(controller.dispose);
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(platform: TargetPlatform.fuchsia),
          home: CustomScrollView(
            controller: controller,
            physics: const NeverScrollableScrollPhysics(),
            slivers: List<Widget>.generate(20, (int index) {
              return SliverToBoxAdapter(
                child: Focus(
                  autofocus: index == 0,
                  child: SizedBox(key: ValueKey<String>('Box $index'), height: 50.0),
                ),
              );
            }),
          ),
        ),
      );

      await tester.pumpAndSettle();
      expect(controller.position.pixels, equals(0.0));
      expect(
        tester.getRect(find.byKey(const ValueKey<String>('Box 0'), skipOffstage: false)),
        equals(const Rect.fromLTRB(0.0, 0.0, 800.0, 50.0)),
      );
      await tester.sendKeyDownEvent(modifierKey);
      await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
      await tester.sendKeyUpEvent(modifierKey);
      await tester.pumpAndSettle();
      expect(controller.position.pixels, equals(0.0));
      expect(
        tester.getRect(find.byKey(const ValueKey<String>('Box 0'), skipOffstage: false)),
        equals(const Rect.fromLTRB(0.0, 0.0, 800.0, 50.0)),
      );
      await tester.sendKeyDownEvent(modifierKey);
      await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
      await tester.sendKeyUpEvent(modifierKey);
      await tester.pumpAndSettle();
      expect(controller.position.pixels, equals(0.0));
      expect(
        tester.getRect(find.byKey(const ValueKey<String>('Box 0'), skipOffstage: false)),
        equals(const Rect.fromLTRB(0.0, 0.0, 800.0, 50.0)),
      );
      await tester.sendKeyEvent(LogicalKeyboardKey.pageDown);
      await tester.pumpAndSettle();
      expect(controller.position.pixels, equals(0.0));
      expect(
        tester.getRect(find.byKey(const ValueKey<String>('Box 0'), skipOffstage: false)),
        equals(const Rect.fromLTRB(0.0, 0.0, 800.0, 50.0)),
      );
      await tester.sendKeyEvent(LogicalKeyboardKey.pageUp);
      await tester.pumpAndSettle();
      expect(controller.position.pixels, equals(0.0));
      expect(
        tester.getRect(find.byKey(const ValueKey<String>('Box 0'), skipOffstage: false)),
        equals(const Rect.fromLTRB(0.0, 0.0, 800.0, 50.0)),
      );
    },
    variant: KeySimulatorTransitModeVariant.all(),
  );

  testWidgets(
    'Vertical scrollables are scrolled when activated via keyboard.',
    (WidgetTester tester) async {
      final ScrollController controller = ScrollController();
      addTearDown(controller.dispose);
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(platform: TargetPlatform.fuchsia),
          home: CustomScrollView(
            controller: controller,
            slivers: List<Widget>.generate(20, (int index) {
              return SliverToBoxAdapter(
                child: Focus(
                  autofocus: index == 0,
                  child: SizedBox(key: ValueKey<String>('Box $index'), height: 50.0),
                ),
              );
            }),
          ),
        ),
      );

      await tester.pumpAndSettle();
      expect(controller.position.pixels, equals(0.0));
      expect(
        tester.getRect(find.byKey(const ValueKey<String>('Box 0'), skipOffstage: false)),
        equals(const Rect.fromLTRB(0.0, 0.0, 800.0, 50.0)),
      );
      // We exclude the modifier keys here for web testing since default web shortcuts
      // do not use a modifier key with arrow keys for ScrollActions.
      if (!kIsWeb) {
        await tester.sendKeyDownEvent(modifierKey);
      }
      await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
      if (!kIsWeb) {
        await tester.sendKeyUpEvent(modifierKey);
      }
      await tester.pumpAndSettle();
      expect(
        tester.getRect(find.byKey(const ValueKey<String>('Box 0'), skipOffstage: false)),
        equals(const Rect.fromLTRB(0.0, -50.0, 800.0, 0.0)),
      );
      if (!kIsWeb) {
        await tester.sendKeyDownEvent(modifierKey);
      }
      await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
      if (!kIsWeb) {
        await tester.sendKeyUpEvent(modifierKey);
      }
      await tester.pumpAndSettle();
      expect(
        tester.getRect(find.byKey(const ValueKey<String>('Box 0'), skipOffstage: false)),
        equals(const Rect.fromLTRB(0.0, 0.0, 800.0, 50.0)),
      );
      await tester.sendKeyEvent(LogicalKeyboardKey.pageDown);
      await tester.pumpAndSettle();
      expect(
        tester.getRect(find.byKey(const ValueKey<String>('Box 0'), skipOffstage: false)),
        equals(const Rect.fromLTRB(0.0, -400.0, 800.0, -350.0)),
      );
      await tester.sendKeyEvent(LogicalKeyboardKey.pageUp);
      await tester.pumpAndSettle();
      expect(
        tester.getRect(find.byKey(const ValueKey<String>('Box 0'), skipOffstage: false)),
        equals(const Rect.fromLTRB(0.0, 0.0, 800.0, 50.0)),
      );
    },
    variant: KeySimulatorTransitModeVariant.all(),
  );

  testWidgets(
    'Horizontal scrollables are scrolled when activated via keyboard.',
    (WidgetTester tester) async {
      final ScrollController controller = ScrollController();
      addTearDown(controller.dispose);
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(platform: TargetPlatform.fuchsia),
          home: CustomScrollView(
            controller: controller,
            scrollDirection: Axis.horizontal,
            slivers: List<Widget>.generate(20, (int index) {
              return SliverToBoxAdapter(
                child: Focus(
                  autofocus: index == 0,
                  child: SizedBox(key: ValueKey<String>('Box $index'), width: 50.0),
                ),
              );
            }),
          ),
        ),
      );

      await tester.pumpAndSettle();
      expect(controller.position.pixels, equals(0.0));
      expect(
        tester.getRect(find.byKey(const ValueKey<String>('Box 0'), skipOffstage: false)),
        equals(const Rect.fromLTRB(0.0, 0.0, 50.0, 600.0)),
      );
      // We exclude the modifier keys here for web testing since default web shortcuts
      // do not use a modifier key with arrow keys for ScrollActions.
      if (!kIsWeb) {
        await tester.sendKeyDownEvent(modifierKey);
      }
      await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
      if (!kIsWeb) {
        await tester.sendKeyUpEvent(modifierKey);
      }
      await tester.pumpAndSettle();
      expect(
        tester.getRect(find.byKey(const ValueKey<String>('Box 0'), skipOffstage: false)),
        equals(const Rect.fromLTRB(-50.0, 0.0, 0.0, 600.0)),
      );
      if (!kIsWeb) {
        await tester.sendKeyDownEvent(modifierKey);
      }
      await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
      if (!kIsWeb) {
        await tester.sendKeyUpEvent(modifierKey);
      }
      await tester.pumpAndSettle();
      expect(
        tester.getRect(find.byKey(const ValueKey<String>('Box 0'), skipOffstage: false)),
        equals(const Rect.fromLTRB(0.0, 0.0, 50.0, 600.0)),
      );
    },
    variant: KeySimulatorTransitModeVariant.all(),
  );

  testWidgets(
    'Horizontal scrollables are scrolled the correct direction in RTL locales.',
    (WidgetTester tester) async {
      final ScrollController controller = ScrollController();
      addTearDown(controller.dispose);
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(platform: TargetPlatform.fuchsia),
          home: Directionality(
            textDirection: TextDirection.rtl,
            child: CustomScrollView(
              controller: controller,
              scrollDirection: Axis.horizontal,
              slivers: List<Widget>.generate(20, (int index) {
                return SliverToBoxAdapter(
                  child: Focus(
                    autofocus: index == 0,
                    child: SizedBox(key: ValueKey<String>('Box $index'), width: 50.0),
                  ),
                );
              }),
            ),
          ),
        ),
      );

      await tester.pumpAndSettle();
      expect(controller.position.pixels, equals(0.0));
      expect(
        tester.getRect(find.byKey(const ValueKey<String>('Box 0'), skipOffstage: false)),
        equals(const Rect.fromLTRB(750.0, 0.0, 800.0, 600.0)),
      );
      // We exclude the modifier keys here for web testing since default web shortcuts
      // do not use a modifier key with arrow keys for ScrollActions.
      if (!kIsWeb) {
        await tester.sendKeyDownEvent(modifierKey);
      }
      await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
      if (!kIsWeb) {
        await tester.sendKeyUpEvent(modifierKey);
      }
      await tester.pumpAndSettle();
      expect(
        tester.getRect(find.byKey(const ValueKey<String>('Box 0'), skipOffstage: false)),
        equals(const Rect.fromLTRB(800.0, 0.0, 850.0, 600.0)),
      );
      if (!kIsWeb) {
        await tester.sendKeyDownEvent(modifierKey);
      }
      await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
      if (!kIsWeb) {
        await tester.sendKeyUpEvent(modifierKey);
      }
      await tester.pumpAndSettle();
      expect(
        tester.getRect(find.byKey(const ValueKey<String>('Box 0'), skipOffstage: false)),
        equals(const Rect.fromLTRB(750.0, 0.0, 800.0, 600.0)),
      );
    },
    variant: KeySimulatorTransitModeVariant.all(),
  );

  testWidgets(
    'Reversed vertical scrollables are scrolled when activated via keyboard.',
    (WidgetTester tester) async {
      final ScrollController controller = ScrollController();
      addTearDown(controller.dispose);
      final FocusNode focusNode = FocusNode(debugLabel: 'SizedBox');
      addTearDown(focusNode.dispose);
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(platform: TargetPlatform.fuchsia),
          home: CustomScrollView(
            controller: controller,
            reverse: true,
            slivers: List<Widget>.generate(20, (int index) {
              return SliverToBoxAdapter(
                child: Focus(
                  focusNode: focusNode,
                  child: SizedBox(key: ValueKey<String>('Box $index'), height: 50.0),
                ),
              );
            }),
          ),
        ),
      );

      focusNode.requestFocus();
      await tester.pumpAndSettle();
      expect(controller.position.pixels, equals(0.0));
      expect(
        tester.getRect(find.byKey(const ValueKey<String>('Box 0'), skipOffstage: false)),
        equals(const Rect.fromLTRB(0.0, 550.0, 800.0, 600.0)),
      );
      // We exclude the modifier keys here for web testing since default web shortcuts
      // do not use a modifier key with arrow keys for ScrollActions.
      if (!kIsWeb) {
        await tester.sendKeyDownEvent(modifierKey);
      }
      await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
      if (!kIsWeb) {
        await tester.sendKeyUpEvent(modifierKey);
      }
      await tester.pumpAndSettle();
      expect(
        tester.getRect(find.byKey(const ValueKey<String>('Box 0'), skipOffstage: false)),
        equals(const Rect.fromLTRB(0.0, 600.0, 800.0, 650.0)),
      );
      if (!kIsWeb) {
        await tester.sendKeyDownEvent(modifierKey);
      }
      await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
      if (!kIsWeb) {
        await tester.sendKeyUpEvent(modifierKey);
      }
      await tester.pumpAndSettle();
      expect(
        tester.getRect(find.byKey(const ValueKey<String>('Box 0'), skipOffstage: false)),
        equals(const Rect.fromLTRB(0.0, 550.0, 800.0, 600.0)),
      );
      await tester.sendKeyEvent(LogicalKeyboardKey.pageUp);
      await tester.pumpAndSettle();
      expect(
        tester.getRect(find.byKey(const ValueKey<String>('Box 0'), skipOffstage: false)),
        equals(const Rect.fromLTRB(0.0, 950.0, 800.0, 1000.0)),
      );
      await tester.sendKeyEvent(LogicalKeyboardKey.pageDown);
      await tester.pumpAndSettle();
      expect(
        tester.getRect(find.byKey(const ValueKey<String>('Box 0'), skipOffstage: false)),
        equals(const Rect.fromLTRB(0.0, 550.0, 800.0, 600.0)),
      );
    },
    variant: KeySimulatorTransitModeVariant.all(),
  );

  testWidgets(
    'Reversed horizontal scrollables are scrolled when activated via keyboard.',
    (WidgetTester tester) async {
      final ScrollController controller = ScrollController();
      addTearDown(controller.dispose);
      final FocusNode focusNode = FocusNode(debugLabel: 'SizedBox');
      addTearDown(focusNode.dispose);
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(platform: TargetPlatform.fuchsia),
          home: CustomScrollView(
            controller: controller,
            scrollDirection: Axis.horizontal,
            reverse: true,
            slivers: List<Widget>.generate(20, (int index) {
              return SliverToBoxAdapter(
                child: Focus(
                  focusNode: focusNode,
                  child: SizedBox(key: ValueKey<String>('Box $index'), width: 50.0),
                ),
              );
            }),
          ),
        ),
      );

      focusNode.requestFocus();
      await tester.pumpAndSettle();
      expect(controller.position.pixels, equals(0.0));
      expect(
        tester.getRect(find.byKey(const ValueKey<String>('Box 0'), skipOffstage: false)),
        equals(const Rect.fromLTRB(750.0, 0.0, 800.0, 600.00)),
      );
      // We exclude the modifier keys here for web testing since default web shortcuts
      // do not use a modifier key with arrow keys for ScrollActions.
      if (!kIsWeb) {
        await tester.sendKeyDownEvent(modifierKey);
      }
      await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
      if (!kIsWeb) {
        await tester.sendKeyUpEvent(modifierKey);
      }
      await tester.pumpAndSettle();
      expect(
        tester.getRect(find.byKey(const ValueKey<String>('Box 0'), skipOffstage: false)),
        equals(const Rect.fromLTRB(800.0, 0.0, 850.0, 600.0)),
      );
      if (!kIsWeb) {
        await tester.sendKeyDownEvent(modifierKey);
      }
      await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
      if (!kIsWeb) {
        await tester.sendKeyUpEvent(modifierKey);
      }
      await tester.pumpAndSettle();
    },
    variant: KeySimulatorTransitModeVariant.all(),
  );

  testWidgets(
    'Custom scrollables with a center sliver are scrolled when activated via keyboard.',
    (WidgetTester tester) async {
      final ScrollController controller = ScrollController();
      addTearDown(controller.dispose);
      final List<String> items = List<String>.generate(20, (int index) => 'Item $index');
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(platform: TargetPlatform.fuchsia),
          home: CustomScrollView(
            controller: controller,
            center: const ValueKey<String>('Center'),
            slivers:
                items.map<Widget>((String item) {
                  return SliverToBoxAdapter(
                    key: item == 'Item 10' ? const ValueKey<String>('Center') : null,
                    child: Focus(
                      autofocus: item == 'Item 10',
                      child: Container(
                        key: ValueKey<String>(item),
                        alignment: Alignment.center,
                        height: 100,
                        child: Text(item),
                      ),
                    ),
                  );
                }).toList(),
          ),
        ),
      );

      await tester.pumpAndSettle();
      expect(controller.position.pixels, equals(0.0));
      expect(
        tester.getRect(find.byKey(const ValueKey<String>('Item 10'), skipOffstage: false)),
        equals(const Rect.fromLTRB(0.0, 0.0, 800.0, 100.0)),
      );
      for (int i = 0; i < 10; ++i) {
        // We exclude the modifier keys here for web testing since default web shortcuts
        // do not use a modifier key with arrow keys for ScrollActions.
        if (!kIsWeb) {
          await tester.sendKeyDownEvent(modifierKey);
        }
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
        if (!kIsWeb) {
          await tester.sendKeyUpEvent(modifierKey);
        }
        await tester.pumpAndSettle();
      }
      // Starts at #10 already, so doesn't work out to 500.0 because it hits bottom.
      expect(controller.position.pixels, equals(400.0));
      expect(
        tester.getRect(find.byKey(const ValueKey<String>('Item 10'), skipOffstage: false)),
        equals(const Rect.fromLTRB(0.0, -400.0, 800.0, -300.0)),
      );
      for (int i = 0; i < 10; ++i) {
        if (!kIsWeb) {
          await tester.sendKeyDownEvent(modifierKey);
        }
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
        if (!kIsWeb) {
          await tester.sendKeyUpEvent(modifierKey);
        }
        await tester.pumpAndSettle();
      }
      // Goes up two past "center" where it started, so negative.
      expect(controller.position.pixels, equals(-100.0));
      expect(
        tester.getRect(find.byKey(const ValueKey<String>('Item 10'), skipOffstage: false)),
        equals(const Rect.fromLTRB(0.0, 100.0, 800.0, 200.0)),
      );
    },
    variant: KeySimulatorTransitModeVariant.all(),
  );

  testWidgets('Can scroll using intents only', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: ListView(
          children: const <Widget>[
            SizedBox(height: 600.0, child: Text('The cow as white as milk')),
            SizedBox(height: 600.0, child: Text('The cape as red as blood')),
            SizedBox(height: 600.0, child: Text('The hair as yellow as corn')),
          ],
        ),
      ),
    );
    expect(find.text('The cow as white as milk'), findsOneWidget);
    expect(find.text('The cape as red as blood'), findsNothing);
    expect(find.text('The hair as yellow as corn'), findsNothing);
    Actions.invoke(
      tester.element(find.byType(SliverList)),
      const ScrollIntent(direction: AxisDirection.down, type: ScrollIncrementType.page),
    );
    await tester.pump(); // start scroll
    await tester.pump(const Duration(milliseconds: 1000)); // end scroll
    expect(find.text('The cow as white as milk'), findsOneWidget);
    expect(find.text('The cape as red as blood'), findsOneWidget);
    expect(find.text('The hair as yellow as corn'), findsNothing);
    Actions.invoke(
      tester.element(find.byType(SliverList)),
      const ScrollIntent(direction: AxisDirection.down, type: ScrollIncrementType.page),
    );
    await tester.pump(); // start scroll
    await tester.pump(const Duration(milliseconds: 1000)); // end scroll
    expect(find.text('The cow as white as milk'), findsNothing);
    expect(find.text('The cape as red as blood'), findsOneWidget);
    expect(find.text('The hair as yellow as corn'), findsOneWidget);
  });

  // Regression test for https://github.com/flutter/flutter/issues/158063.
  testWidgets(
    'Invoking a ScrollAction when notificationContext is null does not cause an exception.',
    (WidgetTester tester) async {
      const List<LogicalKeyboardKey> keysWithModifier = <LogicalKeyboardKey>[
        LogicalKeyboardKey.arrowDown,
        LogicalKeyboardKey.arrowUp,
      ];
      const List<LogicalKeyboardKey> keys = <LogicalKeyboardKey>[
        ...keysWithModifier,
        LogicalKeyboardKey.pageDown,
        LogicalKeyboardKey.pageUp,
      ];
      final ScrollController controller = ScrollController();
      addTearDown(controller.dispose);
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(platform: TargetPlatform.fuchsia),
          home: PrimaryScrollController(
            controller: controller,
            child: Focus(
              autofocus: true,
              child: _NoNotificationContextScrollable(
                controller: controller,
                viewportBuilder:
                    (BuildContext context, ViewportOffset offset) => Viewport(
                      offset: offset,
                      slivers: List<Widget>.generate(
                        20,
                        (int index) => SliverToBoxAdapter(
                          child: SizedBox(key: ValueKey<String>('Box $index'), height: 50.0),
                        ),
                      ),
                    ),
              ),
            ),
          ),
        ),
      );

      // Verify the initial scroll offset.
      await tester.pumpAndSettle();
      expect(controller.position.pixels, equals(0.0));
      expect(
        tester.getRect(find.byKey(const ValueKey<String>('Box 0'), skipOffstage: false)),
        equals(const Rect.fromLTRB(0.0, 0.0, 800.0, 50.0)),
      );

      for (final LogicalKeyboardKey key in keys) {
        // The default web shortcuts do not use a modifier key for ScrollActions.
        if (!kIsWeb && keysWithModifier.contains(key)) {
          await tester.sendKeyDownEvent(modifierKey);
        }

        await tester.sendKeyEvent(key);
        expect(tester.takeException(), isNull);

        if (!kIsWeb && keysWithModifier.contains(key)) {
          await tester.sendKeyUpEvent(modifierKey);
        }

        // No scrollable is found, so the scroll position should not change.
        await tester.pumpAndSettle();
        expect(controller.position.pixels, equals(0.0));
        expect(
          tester.getRect(find.byKey(const ValueKey<String>('Box 0'), skipOffstage: false)),
          equals(const Rect.fromLTRB(0.0, 0.0, 800.0, 50.0)),
        );
      }
    },
    variant: KeySimulatorTransitModeVariant.all(),
  );
}
