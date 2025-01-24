// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

import '../rendering/rendering_tester.dart';
import '../widgets/semantics_tester.dart';

class SpyFixedExtentScrollController extends FixedExtentScrollController {
  /// Override for test visibility only.
  @override
  bool get hasListeners => super.hasListeners;
}

void main() {
  testWidgets('Picker respects theme styling', (WidgetTester tester) async {
    await tester.pumpWidget(
      CupertinoApp(
        home: Align(
          alignment: Alignment.topLeft,
          child: SizedBox(
            height: 300.0,
            width: 300.0,
            child: CupertinoPicker(
              itemExtent: 50.0,
              onSelectedItemChanged: (_) {},
              children: List<Widget>.generate(3, (int index) {
                return SizedBox(height: 50.0, width: 300.0, child: Text(index.toString()));
              }),
            ),
          ),
        ),
      ),
    );

    final RenderParagraph paragraph = tester.renderObject(find.text('1'));

    expect(paragraph.text.style!.color, isSameColorAs(CupertinoColors.black));
    expect(
      paragraph.text.style!.copyWith(color: CupertinoColors.black),
      const TextStyle(
        inherit: false,
        fontFamily: 'CupertinoSystemDisplay',
        fontSize: 21.0,
        fontWeight: FontWeight.w400,
        letterSpacing: -0.6,
        color: CupertinoColors.black,
      ),
    );
  });

  testWidgets('Picker semantics', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);

    await tester.pumpWidget(
      CupertinoApp(
        home: SizedBox(
          height: 300.0,
          width: 300.0,
          child: CupertinoPicker(
            itemExtent: 50.0,
            onSelectedItemChanged: (_) {},
            children: List<Widget>.generate(13, (int index) {
              return SizedBox(height: 50.0, width: 300.0, child: Text(index.toString()));
            }),
          ),
        ),
      ),
    );
    expect(
      semantics,
      includesNodeWith(
        value: '0',
        increasedValue: '1',
        actions: <SemanticsAction>[SemanticsAction.increase],
      ),
    );

    final FixedExtentScrollController hourListController =
        tester.widget<ListWheelScrollView>(find.byType(ListWheelScrollView)).controller!
            as FixedExtentScrollController;

    hourListController.jumpToItem(11);
    await tester.pumpAndSettle();
    expect(
      semantics,
      includesNodeWith(
        value: '11',
        increasedValue: '12',
        decreasedValue: '10',
        actions: <SemanticsAction>[SemanticsAction.increase, SemanticsAction.decrease],
      ),
    );
    semantics.dispose();
  });

  group('layout', () {
    // Regression test for https://github.com/flutter/flutter/issues/22999
    testWidgets('CupertinoPicker.builder test', (WidgetTester tester) async {
      Widget buildFrame(int childCount) {
        return Directionality(
          textDirection: TextDirection.ltr,
          child: CupertinoPicker.builder(
            itemExtent: 50.0,
            onSelectedItemChanged: (_) {},
            itemBuilder: (BuildContext context, int index) {
              return Text('$index');
            },
            childCount: childCount,
          ),
        );
      }

      await tester.pumpWidget(buildFrame(1));
      expect(tester.renderObject(find.text('0')).attached, true);

      await tester.pumpWidget(buildFrame(2));
      expect(tester.renderObject(find.text('0')).attached, true);
      expect(tester.renderObject(find.text('1')).attached, true);
    });

    testWidgets('selected item is in the middle', (WidgetTester tester) async {
      final FixedExtentScrollController controller = FixedExtentScrollController(initialItem: 1);
      addTearDown(controller.dispose);
      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: Align(
            alignment: Alignment.topLeft,
            child: SizedBox(
              height: 300.0,
              width: 300.0,
              child: CupertinoPicker(
                scrollController: controller,
                itemExtent: 50.0,
                onSelectedItemChanged: (_) {},
                children: List<Widget>.generate(3, (int index) {
                  return SizedBox(height: 50.0, width: 300.0, child: Text(index.toString()));
                }),
              ),
            ),
          ),
        ),
      );

      expect(tester.getTopLeft(find.widgetWithText(SizedBox, '1').first), const Offset(0.0, 125.0));

      controller.jumpToItem(0);
      await tester.pump();

      expect(
        tester.getTopLeft(find.widgetWithText(SizedBox, '1').first),
        offsetMoreOrLessEquals(const Offset(0.0, 170.0), epsilon: 0.5),
      );
      expect(tester.getTopLeft(find.widgetWithText(SizedBox, '0').first), const Offset(0.0, 125.0));
    });
  });

  testWidgets('picker dark mode', (WidgetTester tester) async {
    await tester.pumpWidget(
      CupertinoApp(
        theme: const CupertinoThemeData(brightness: Brightness.light),
        home: Align(
          alignment: Alignment.topLeft,
          child: SizedBox(
            height: 300.0,
            width: 300.0,
            child: CupertinoPicker(
              backgroundColor: const CupertinoDynamicColor.withBrightness(
                color: Color(
                  0xFF123456,
                ), // Set alpha channel to FF to disable under magnifier painting.
                darkColor: Color(0xFF654321),
              ),
              itemExtent: 15.0,
              children: const <Widget>[Text('1'), Text('1')],
              onSelectedItemChanged: (int i) {},
            ),
          ),
        ),
      ),
    );

    expect(
      find.byType(CupertinoPicker),
      paints..rrect(color: const Color.fromARGB(30, 118, 118, 128)),
    );
    expect(find.byType(CupertinoPicker), paints..rect(color: const Color(0xFF123456)));

    await tester.pumpWidget(
      CupertinoApp(
        theme: const CupertinoThemeData(brightness: Brightness.dark),
        home: Align(
          alignment: Alignment.topLeft,
          child: SizedBox(
            height: 300.0,
            width: 300.0,
            child: CupertinoPicker(
              backgroundColor: const CupertinoDynamicColor.withBrightness(
                color: Color(0xFF123456),
                darkColor: Color(0xFF654321),
              ),
              itemExtent: 15.0,
              children: const <Widget>[Text('1'), Text('1')],
              onSelectedItemChanged: (int i) {},
            ),
          ),
        ),
      ),
    );

    expect(
      find.byType(CupertinoPicker),
      paints..rrect(color: const Color.fromARGB(61, 118, 118, 128)),
    );
    expect(find.byType(CupertinoPicker), paints..rect(color: const Color(0xFF654321)));
  });

  testWidgets('picker selectionOverlay', (WidgetTester tester) async {
    await tester.pumpWidget(
      CupertinoApp(
        theme: const CupertinoThemeData(brightness: Brightness.light),
        home: Align(
          alignment: Alignment.topLeft,
          child: SizedBox(
            height: 300.0,
            width: 300.0,
            child: CupertinoPicker(
              itemExtent: 15.0,
              onSelectedItemChanged: (int i) {},
              selectionOverlay: const CupertinoPickerDefaultSelectionOverlay(
                background: Color(0x12345678),
              ),
              children: const <Widget>[Text('1'), Text('1')],
            ),
          ),
        ),
      ),
    );

    expect(find.byType(CupertinoPicker), paints..rrect(color: const Color(0x12345678)));
  });

  testWidgets('CupertinoPicker.selectionOverlay is nullable', (WidgetTester tester) async {
    await tester.pumpWidget(
      CupertinoApp(
        theme: const CupertinoThemeData(brightness: Brightness.light),
        home: Align(
          alignment: Alignment.topLeft,
          child: SizedBox(
            height: 300.0,
            width: 300.0,
            child: CupertinoPicker(
              itemExtent: 15.0,
              onSelectedItemChanged: (int i) {},
              selectionOverlay: null,
              children: const <Widget>[Text('1'), Text('1')],
            ),
          ),
        ),
      ),
    );

    expect(find.byType(CupertinoPicker), isNot(paints..rrect()));
  });

  group('scroll', () {
    testWidgets(
      'scrolling calls onSelectedItemChanged and triggers haptic feedback',
      (WidgetTester tester) async {
        final List<int> selectedItems = <int>[];
        final List<MethodCall> systemCalls = <MethodCall>[];

        tester.binding.defaultBinaryMessenger.setMockMethodCallHandler(SystemChannels.platform, (
          MethodCall methodCall,
        ) async {
          systemCalls.add(methodCall);
          return null;
        });

        await tester.pumpWidget(
          Directionality(
            textDirection: TextDirection.ltr,
            child: CupertinoPicker(
              itemExtent: 100.0,
              onSelectedItemChanged: (int index) {
                selectedItems.add(index);
              },
              children: List<Widget>.generate(100, (int index) {
                return Center(
                  child: SizedBox(width: 400.0, height: 100.0, child: Text(index.toString())),
                );
              }),
            ),
          ),
        );

        await tester.drag(
          find.text('0'),
          const Offset(0.0, -100.0),
          warnIfMissed: false,
        ); // has an IgnorePointer
        expect(selectedItems, <int>[1]);
        expect(
          systemCalls.single,
          isMethodCall('HapticFeedback.vibrate', arguments: 'HapticFeedbackType.selectionClick'),
        );

        await tester.drag(
          find.text('0'),
          const Offset(0.0, 100.0),
          warnIfMissed: false,
        ); // has an IgnorePointer
        expect(selectedItems, <int>[1, 0]);
        expect(systemCalls, hasLength(2));
        expect(
          systemCalls.last,
          isMethodCall('HapticFeedback.vibrate', arguments: 'HapticFeedbackType.selectionClick'),
        );
      },
      variant: TargetPlatformVariant.only(TargetPlatform.iOS),
    );

    testWidgets(
      'do not trigger haptic effects on non-iOS devices',
      (WidgetTester tester) async {
        final List<int> selectedItems = <int>[];
        final List<MethodCall> systemCalls = <MethodCall>[];

        tester.binding.defaultBinaryMessenger.setMockMethodCallHandler(SystemChannels.platform, (
          MethodCall methodCall,
        ) async {
          systemCalls.add(methodCall);
          return null;
        });

        await tester.pumpWidget(
          Directionality(
            textDirection: TextDirection.ltr,
            child: CupertinoPicker(
              itemExtent: 100.0,
              onSelectedItemChanged: (int index) {
                selectedItems.add(index);
              },
              children: List<Widget>.generate(100, (int index) {
                return Center(
                  child: SizedBox(width: 400.0, height: 100.0, child: Text(index.toString())),
                );
              }),
            ),
          ),
        );

        await tester.drag(
          find.text('0'),
          const Offset(0.0, -100.0),
          warnIfMissed: false,
        ); // has an IgnorePointer
        expect(selectedItems, <int>[1]);
        expect(systemCalls, isEmpty);
      },
      variant: TargetPlatformVariant(
        TargetPlatform.values
            .where((TargetPlatform platform) => platform != TargetPlatform.iOS)
            .toSet(),
      ),
    );

    testWidgets(
      'a drag in between items settles back',
      (WidgetTester tester) async {
        final FixedExtentScrollController controller = FixedExtentScrollController(initialItem: 10);
        addTearDown(controller.dispose);
        final List<int> selectedItems = <int>[];

        await tester.pumpWidget(
          Directionality(
            textDirection: TextDirection.ltr,
            child: CupertinoPicker(
              scrollController: controller,
              itemExtent: 100.0,
              onSelectedItemChanged: (int index) {
                selectedItems.add(index);
              },
              children: List<Widget>.generate(100, (int index) {
                return Center(
                  child: SizedBox(width: 400.0, height: 100.0, child: Text(index.toString())),
                );
              }),
            ),
          ),
        );

        // Drag it by a bit but not enough to move to the next item.
        await tester.drag(
          find.text('10'),
          const Offset(0.0, 30.0),
          pointer: 1,
          touchSlopY: 0.0,
          warnIfMissed: false,
        ); // has an IgnorePointer

        // The item that was in the center now moved a bit.
        expect(tester.getTopLeft(find.widgetWithText(SizedBox, '10')), const Offset(200.0, 250.0));

        await tester.pumpAndSettle();

        expect(
          tester.getTopLeft(find.widgetWithText(SizedBox, '10')).dy,
          moreOrLessEquals(250.0, epsilon: 0.5),
        );
        expect(selectedItems.isEmpty, true);

        // Drag it by enough to move to the next item.
        await tester.drag(
          find.text('10'),
          const Offset(0.0, 70.0),
          pointer: 1,
          touchSlopY: 0.0,
          warnIfMissed: false,
        ); // has an IgnorePointer

        await tester.pumpAndSettle();

        expect(
          tester.getTopLeft(find.widgetWithText(SizedBox, '10')).dy,
          // It's down by 100.0 now.
          moreOrLessEquals(340.0, epsilon: 0.5),
        );
        expect(selectedItems, <int>[9]);
      },
      variant: const TargetPlatformVariant(<TargetPlatform>{
        TargetPlatform.iOS,
        TargetPlatform.macOS,
      }),
    );

    testWidgets(
      'a big fling that overscrolls springs back',
      (WidgetTester tester) async {
        final FixedExtentScrollController controller = FixedExtentScrollController(initialItem: 10);
        addTearDown(controller.dispose);
        final List<int> selectedItems = <int>[];

        await tester.pumpWidget(
          Directionality(
            textDirection: TextDirection.ltr,
            child: CupertinoPicker(
              scrollController: controller,
              itemExtent: 100.0,
              onSelectedItemChanged: (int index) {
                selectedItems.add(index);
              },
              children: List<Widget>.generate(100, (int index) {
                return Center(
                  child: SizedBox(width: 400.0, height: 100.0, child: Text(index.toString())),
                );
              }),
            ),
          ),
        );

        // A wild throw appears.
        await tester.fling(
          find.text('10'),
          const Offset(0.0, 10000.0),
          1000.0,
          warnIfMissed: false, // has an IgnorePointer
        );

        if (debugDefaultTargetPlatformOverride == TargetPlatform.iOS) {
          // Should have been flung far enough that even the first item goes off
          // screen and gets removed.
          expect(find.widgetWithText(SizedBox, '0').evaluate().isEmpty, true);
        }

        expect(
          selectedItems,
          // This specific throw was fast enough that each scroll update landed
          // on every second item.
          <int>[8, 6, 4, 2, 0],
        );

        // Let it spring back.
        await tester.pumpAndSettle();

        expect(
          tester.getTopLeft(find.widgetWithText(SizedBox, '0')).dy,
          // Should have sprung back to the middle now.
          moreOrLessEquals(250.0),
        );
        expect(
          selectedItems,
          // Falling back to 0 shouldn't produce more callbacks.
          <int>[8, 6, 4, 2, 0],
        );
      },
      variant: const TargetPlatformVariant(<TargetPlatform>{
        TargetPlatform.iOS,
        TargetPlatform.macOS,
      }),
    );
  });

  testWidgets('Picker adapts to MaterialApp dark mode', (WidgetTester tester) async {
    Widget buildCupertinoPicker(Brightness brightness) {
      return MaterialApp(
        theme: ThemeData(brightness: brightness),
        home: Align(
          alignment: Alignment.topLeft,
          child: SizedBox(
            height: 300.0,
            width: 300.0,
            child: CupertinoPicker(
              itemExtent: 50.0,
              onSelectedItemChanged: (_) {},
              children: List<Widget>.generate(3, (int index) {
                return SizedBox(height: 50.0, width: 300.0, child: Text(index.toString()));
              }),
            ),
          ),
        ),
      );
    }

    // CupertinoPicker with light theme.
    await tester.pumpWidget(buildCupertinoPicker(Brightness.light));
    RenderParagraph paragraph = tester.renderObject(find.text('1'));
    expect(paragraph.text.style!.color, CupertinoColors.label);
    // Text style should not return unresolved color.
    expect(paragraph.text.style!.color.toString().contains('UNRESOLVED'), isFalse);

    // CupertinoPicker with dark theme.
    await tester.pumpWidget(buildCupertinoPicker(Brightness.dark));
    paragraph = tester.renderObject(find.text('1'));
    expect(paragraph.text.style!.color, CupertinoColors.label);
    // Text style should not return unresolved color.
    expect(paragraph.text.style!.color.toString().contains('UNRESOLVED'), isFalse);
  });

  group('CupertinoPickerDefaultSelectionOverlay', () {
    testWidgets('should be using directional decoration', (WidgetTester tester) async {
      await tester.pumpWidget(
        CupertinoApp(
          theme: const CupertinoThemeData(brightness: Brightness.light),
          home: CupertinoPicker(
            itemExtent: 15.0,
            onSelectedItemChanged: (int i) {},
            selectionOverlay: const CupertinoPickerDefaultSelectionOverlay(
              background: Color(0x12345678),
            ),
            children: const <Widget>[Text('1'), Text('1')],
          ),
        ),
      );

      final Finder selectionContainer = find.byType(Container);
      final Container container = tester.firstWidget<Container>(selectionContainer);
      final EdgeInsetsGeometry? margin = container.margin;
      final BorderRadiusGeometry? borderRadius =
          (container.decoration as BoxDecoration?)?.borderRadius;

      expect(margin, isA<EdgeInsetsDirectional>());
      expect(borderRadius, isA<BorderRadiusDirectional>());
    });
  });

  testWidgets('Scroll controller is detached upon dispose', (WidgetTester tester) async {
    final SpyFixedExtentScrollController controller = SpyFixedExtentScrollController();
    addTearDown(controller.dispose);
    expect(controller.hasListeners, false);
    expect(controller.positions.length, 0);

    await tester.pumpWidget(
      CupertinoApp(
        home: Align(
          alignment: Alignment.topLeft,
          child: Center(
            child: CupertinoPicker(
              scrollController: controller,
              itemExtent: 50.0,
              onSelectedItemChanged: (_) {},
              children: List<Widget>.generate(3, (int index) {
                return SizedBox(width: 300.0, child: Text(index.toString()));
              }),
            ),
          ),
        ),
      ),
    );
    expect(controller.hasListeners, true);
    expect(controller.positions.length, 1);

    await tester.pumpWidget(const SizedBox.expand());
    expect(controller.hasListeners, false);
    expect(controller.positions.length, 0);
  });

  testWidgets('Registers taps and does not crash with certain diameterRatio', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/126491

    final List<int> children = List<int>.generate(100, (int index) => index);
    final List<int> paintedChildren = <int>[];
    final Set<int> tappedChildren = <int>{};

    await tester.pumpWidget(
      CupertinoApp(
        home: Align(
          alignment: Alignment.topLeft,
          child: Center(
            child: SizedBox(
              height: 120,
              child: CupertinoPicker(
                itemExtent: 55,
                diameterRatio: 0.9,
                onSelectedItemChanged: (int index) {},
                children:
                    children
                        .map<Widget>(
                          (int index) => GestureDetector(
                            key: ValueKey<int>(index),
                            onTap: () {
                              tappedChildren.add(index);
                            },
                            child: SizedBox(
                              width: 55,
                              height: 55,
                              child: CustomPaint(
                                painter: TestCallbackPainter(
                                  onPaint: () {
                                    paintedChildren.add(index);
                                  },
                                ),
                              ),
                            ),
                          ),
                        )
                        .toList(),
              ),
            ),
          ),
        ),
      ),
    );

    // Children are painted two times for whatever reason
    expect(paintedChildren, <int>[0, 1, 0, 1]);

    // Expect hitting 0 and 1, which are painted
    await tester.tap(find.byKey(const ValueKey<int>(0)));
    expect(tappedChildren, const <int>[0]);

    await tester.tap(find.byKey(const ValueKey<int>(1)));
    expect(tappedChildren, const <int>[0, 1]);

    // The third child is not painted, so is not hit
    await tester.tap(find.byKey(const ValueKey<int>(2)), warnIfMissed: false);
    expect(tappedChildren, const <int>[0, 1]);
  });

  testWidgets('Tapping on child in a CupertinoPicker selects that child', (
    WidgetTester tester,
  ) async {
    int selectedItem = 0;
    const Duration tapScrollDuration = Duration(milliseconds: 300);
    // The tap animation is set to 300ms, but add an extra 1µs to complete the scroll animation.
    const Duration infinitesimalPause = Duration(microseconds: 1);

    await tester.pumpWidget(
      CupertinoApp(
        home: CupertinoPicker(
          itemExtent: 10.0,
          onSelectedItemChanged: (int i) {
            selectedItem = i;
          },
          children: const <Widget>[Text('0'), Text('1'), Text('2'), Text('3')],
        ),
      ),
    );

    expect(selectedItem, equals(0));
    // Tap on the item at index 1.
    await tester.tap(find.text('1'));
    await tester.pump();
    await tester.pump(tapScrollDuration + infinitesimalPause);
    expect(selectedItem, equals(1));

    // Skip to the item at index 3.
    await tester.tap(find.text('3'));
    await tester.pump();
    await tester.pump(tapScrollDuration + infinitesimalPause);
    expect(selectedItem, equals(3));

    // Tap on the item at index 0.
    await tester.tap(find.text('0'));
    await tester.pump();
    await tester.pump(tapScrollDuration + infinitesimalPause);
    expect(selectedItem, equals(0));

    // Skip to the item at index 2.
    await tester.tap(find.text('2'));
    await tester.pump();
    await tester.pump(tapScrollDuration + infinitesimalPause);
    expect(selectedItem, equals(2));
  });
}
