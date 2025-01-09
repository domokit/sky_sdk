// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui' as ui;

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

// The const represents the starting position of the scrollbar thumb for
// the below tests. The thumb is 90 pixels long, and 8 pixels wide, with a 2
// pixel margin to the right edge of the viewport.
const Rect _kMaterialDesignInitialThumbRect = Rect.fromLTRB(790.0, 0.0, 798.0, 90.0);
const Radius _kDefaultThumbRadius = Radius.circular(8.0);
const Color _kDefaultIdleThumbColor = Color(0x1a000000);
const Color _kDefaultDragThumbColor = Color(0x99000000);

void main() {
  test('ScrollbarThemeData copyWith, ==, hashCode basics', () {
    expect(const ScrollbarThemeData(), const ScrollbarThemeData().copyWith());
    expect(const ScrollbarThemeData().hashCode, const ScrollbarThemeData().copyWith().hashCode);
  });

  test('ScrollbarThemeData lerp special cases', () {
    expect(ScrollbarThemeData.lerp(null, null, 0), const ScrollbarThemeData());
    const ScrollbarThemeData data = ScrollbarThemeData();
    expect(identical(ScrollbarThemeData.lerp(data, data, 0.5), data), true);
  });

  testWidgets(
    'Passing no ScrollbarTheme returns defaults',
    (WidgetTester tester) async {
      final ScrollController scrollController = ScrollController();
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(
            useMaterial3: false,
            scrollbarTheme: ScrollbarThemeData(
              trackVisibility: MaterialStateProperty.resolveWith((Set<MaterialState> states) {
                if (states.contains(MaterialState.hovered)) {
                  return true;
                }
                return false;
              }),
            ),
          ),
          home: ScrollConfiguration(
            behavior: const NoScrollbarBehavior(),
            child: Scrollbar(
              thumbVisibility: true,
              controller: scrollController,
              child: SingleChildScrollView(
                controller: scrollController,
                child: const SizedBox(width: 4000.0, height: 4000.0),
              ),
            ),
          ),
        ),
      );
      await tester.pumpAndSettle();
      // Idle scrollbar behavior
      expect(
        find.byType(Scrollbar),
        paints..rrect(
          rrect: RRect.fromRectAndRadius(_kMaterialDesignInitialThumbRect, _kDefaultThumbRadius),
          color: _kDefaultIdleThumbColor,
        ),
      );

      // Drag scrollbar behavior
      const double scrollAmount = 10.0;
      final TestGesture dragScrollbarGesture = await tester.startGesture(const Offset(797.0, 45.0));
      await tester.pumpAndSettle();

      expect(
        find.byType(Scrollbar),
        paints..rrect(
          rrect: RRect.fromRectAndRadius(_kMaterialDesignInitialThumbRect, _kDefaultThumbRadius),
          // Drag color
          color: _kDefaultDragThumbColor,
        ),
      );

      await dragScrollbarGesture.moveBy(const Offset(0.0, scrollAmount));
      await tester.pumpAndSettle();
      await dragScrollbarGesture.up();
      await tester.pumpAndSettle();

      // Hover scrollbar behavior
      final TestGesture gesture = await tester.createGesture(kind: ui.PointerDeviceKind.mouse);
      await gesture.addPointer();
      await gesture.moveTo(const Offset(794.0, 5.0));
      await tester.pumpAndSettle();

      expect(
        find.byType(Scrollbar),
        paints
          ..rect(
            rect: const Rect.fromLTRB(784.0, 0.0, 800.0, 600.0),
            color: const Color(0x08000000),
          )
          ..line(
            p1: const Offset(784.0, 0.0),
            p2: const Offset(784.0, 600.0),
            strokeWidth: 1.0,
            color: const Color(0x1a000000),
          )
          ..rrect(
            rrect: RRect.fromRectAndRadius(
              // Scrollbar thumb is larger
              const Rect.fromLTRB(786.0, 10.0, 798.0, 100.0),
              _kDefaultThumbRadius,
            ),
            // Hover color
            color: const Color(0x80000000),
          ),
      );

      scrollController.dispose();
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.linux,
      TargetPlatform.macOS,
      TargetPlatform.windows,
      TargetPlatform.fuchsia,
    }),
  );

  testWidgets(
    'Scrollbar uses values from ScrollbarTheme',
    (WidgetTester tester) async {
      final ScrollbarThemeData scrollbarTheme = _scrollbarTheme();
      final ScrollController scrollController = ScrollController();
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(scrollbarTheme: scrollbarTheme),
          home: ScrollConfiguration(
            behavior: const NoScrollbarBehavior(),
            child: Scrollbar(
              thumbVisibility: true,
              controller: scrollController,
              child: SingleChildScrollView(
                controller: scrollController,
                child: const SizedBox(width: 4000.0, height: 4000.0),
              ),
            ),
          ),
        ),
      );
      await tester.pumpAndSettle();
      // Idle scrollbar behavior
      expect(
        find.byType(Scrollbar),
        paints..rrect(
          rrect: RRect.fromRectAndRadius(
            const Rect.fromLTRB(785.0, 10.0, 795.0, 97.0),
            const Radius.circular(6.0),
          ),
          color: const Color(0xff4caf50),
        ),
      );

      // Drag scrollbar behavior
      const double scrollAmount = 10.0;
      final TestGesture dragScrollbarGesture = await tester.startGesture(const Offset(797.0, 45.0));
      await tester.pumpAndSettle();

      expect(
        find.byType(Scrollbar),
        paints..rrect(
          rrect: RRect.fromRectAndRadius(
            const Rect.fromLTRB(785.0, 10.0, 795.0, 97.0),
            const Radius.circular(6.0),
          ),
          // Drag color
          color: const Color(0xfff44336),
        ),
      );

      await dragScrollbarGesture.moveBy(const Offset(0.0, scrollAmount));
      await tester.pumpAndSettle();
      await dragScrollbarGesture.up();
      await tester.pumpAndSettle();

      // Hover scrollbar behavior
      final TestGesture gesture = await tester.createGesture(kind: ui.PointerDeviceKind.mouse);
      await gesture.addPointer();
      await gesture.moveTo(const Offset(794.0, 15.0));
      await tester.pumpAndSettle();

      expect(
        find.byType(Scrollbar),
        paints
          ..rect(
            rect: const Rect.fromLTRB(770.0, 0.0, 800.0, 600.0),
            color: const Color(0xff000000),
          )
          ..line(
            p1: const Offset(770.0, 00.0),
            p2: const Offset(770.0, 600.0),
            strokeWidth: 1.0,
            color: const Color(0xffffeb3b),
          )
          ..rrect(
            rrect: RRect.fromRectAndRadius(
              // Scrollbar thumb is larger
              const Rect.fromLTRB(775.0, 20.0, 795.0, 107.0),
              const Radius.circular(6.0),
            ),
            // Hover color
            color: const Color(0xff2196f3),
          ),
      );

      scrollController.dispose();
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.linux,
      TargetPlatform.macOS,
      TargetPlatform.windows,
      TargetPlatform.fuchsia,
    }),
  );

  testWidgets('Scrollbar uses values from ScrollbarTheme if exists instead of values from Theme', (
    WidgetTester tester,
  ) async {
    final ScrollbarThemeData scrollbarTheme = _scrollbarTheme();
    final ScrollController scrollController = ScrollController();
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(scrollbarTheme: scrollbarTheme),
        home: ScrollConfiguration(
          behavior: const NoScrollbarBehavior(),
          child: ScrollbarTheme(
            data: _scrollbarTheme().copyWith(
              thumbColor: MaterialStateProperty.all(const Color(0xFF000000)),
            ),
            child: Scrollbar(
              thumbVisibility: true,
              controller: scrollController,
              child: SingleChildScrollView(
                controller: scrollController,
                child: const SizedBox(width: 4000.0, height: 4000.0),
              ),
            ),
          ),
        ),
      ),
    );
    await tester.pumpAndSettle();
    // Idle scrollbar behavior
    expect(
      find.byType(Scrollbar),
      paints..rrect(
        rrect: RRect.fromRectAndRadius(
          const Rect.fromLTRB(785.0, 10.0, 795.0, 97.0),
          const Radius.circular(6.0),
        ),
        color: const Color(0xFF000000),
      ),
    );

    scrollController.dispose();
  });

  testWidgets(
    'ScrollbarTheme can disable gestures',
    (WidgetTester tester) async {
      final ScrollController scrollController = ScrollController();
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(
            useMaterial3: false,
            scrollbarTheme: const ScrollbarThemeData(interactive: false),
          ),
          home: Scrollbar(
            thumbVisibility: true,
            controller: scrollController,
            child: SingleChildScrollView(
              controller: scrollController,
              child: const SizedBox(width: 4000.0, height: 4000.0),
            ),
          ),
        ),
      );
      await tester.pumpAndSettle();
      // Idle scrollbar behavior
      expect(
        find.byType(Scrollbar),
        paints..rrect(
          rrect: RRect.fromRectAndRadius(_kMaterialDesignInitialThumbRect, _kDefaultThumbRadius),
          color: _kDefaultIdleThumbColor,
        ),
      );

      // Try to drag scrollbar.
      const double scrollAmount = 10.0;
      final TestGesture dragScrollbarGesture = await tester.startGesture(const Offset(797.0, 45.0));
      await tester.pumpAndSettle();
      await dragScrollbarGesture.moveBy(const Offset(0.0, scrollAmount));
      await tester.pumpAndSettle();
      await dragScrollbarGesture.up();
      await tester.pumpAndSettle();
      // Expect no change
      expect(
        find.byType(Scrollbar),
        paints..rrect(
          rrect: RRect.fromRectAndRadius(_kMaterialDesignInitialThumbRect, _kDefaultThumbRadius),
          color: _kDefaultIdleThumbColor,
        ),
      );

      scrollController.dispose();
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.fuchsia}),
  );

  testWidgets(
    'Scrollbar.interactive takes priority over ScrollbarTheme',
    (WidgetTester tester) async {
      final ScrollController scrollController = ScrollController();
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(
            useMaterial3: false,
            scrollbarTheme: const ScrollbarThemeData(interactive: false),
          ),
          home: Scrollbar(
            interactive: true,
            thumbVisibility: true,
            controller: scrollController,
            child: SingleChildScrollView(
              controller: scrollController,
              child: const SizedBox(width: 4000.0, height: 4000.0),
            ),
          ),
        ),
      );
      await tester.pumpAndSettle();
      // Idle scrollbar behavior
      expect(
        find.byType(Scrollbar),
        paints..rrect(
          rrect: RRect.fromRectAndRadius(_kMaterialDesignInitialThumbRect, _kDefaultThumbRadius),
          color: _kDefaultIdleThumbColor,
        ),
      );

      // Drag scrollbar.
      const double scrollAmount = 10.0;
      final TestGesture dragScrollbarGesture = await tester.startGesture(const Offset(797.0, 45.0));
      await tester.pumpAndSettle();
      await dragScrollbarGesture.moveBy(const Offset(0.0, scrollAmount));
      await tester.pumpAndSettle();
      await dragScrollbarGesture.up();
      await tester.pumpAndSettle();
      // Gestures handled by Scrollbar.
      expect(
        find.byType(Scrollbar),
        paints..rrect(
          rrect: RRect.fromRectAndRadius(
            const Rect.fromLTRB(790.0, 10.0, 798.0, 100.0),
            _kDefaultThumbRadius,
          ),
          color: _kDefaultIdleThumbColor,
        ),
      );

      scrollController.dispose();
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.fuchsia}),
  );

  testWidgets(
    'Scrollbar widget properties take priority over theme',
    (WidgetTester tester) async {
      const double thickness = 4.0;
      const double edgeMargin = 2.0;
      const Radius radius = Radius.circular(3.0);
      final ScrollController scrollController = ScrollController();

      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(
            colorScheme: const ColorScheme.light(),
            scrollbarTheme: ScrollbarThemeData(
              trackVisibility: MaterialStateProperty.resolveWith((Set<MaterialState> states) {
                if (states.contains(MaterialState.hovered)) {
                  return true;
                }
                return false;
              }),
            ),
          ),
          home: ScrollConfiguration(
            behavior: const NoScrollbarBehavior(),
            child: Scrollbar(
              thickness: thickness,
              thumbVisibility: true,
              radius: radius,
              controller: scrollController,
              child: SingleChildScrollView(
                controller: scrollController,
                child: const SizedBox(width: 4000.0, height: 4000.0),
              ),
            ),
          ),
        ),
      );

      await tester.pumpAndSettle();
      // Idle scrollbar behavior
      expect(
        find.byType(Scrollbar),
        paints..rrect(
          rrect: RRect.fromRectAndRadius(
            const Rect.fromLTRB(800 - thickness - edgeMargin, 0.0, 798.0, 90.0),
            const Radius.circular(3.0),
          ),
          color: _kDefaultIdleThumbColor,
        ),
      );

      // Drag scrollbar behavior.
      const double scrollAmount = 10.0;
      final TestGesture dragScrollbarGesture = await tester.startGesture(const Offset(797.0, 45.0));
      await tester.pumpAndSettle();

      expect(
        find.byType(Scrollbar),
        paints..rrect(
          rrect: RRect.fromRectAndRadius(
            const Rect.fromLTRB(800 - thickness - edgeMargin, 0.0, 798.0, 90.0),
            const Radius.circular(3.0),
          ),
          // Drag color
          color: _kDefaultDragThumbColor,
        ),
      );

      await dragScrollbarGesture.moveBy(const Offset(0.0, scrollAmount));
      await tester.pumpAndSettle();
      await dragScrollbarGesture.up();
      await tester.pumpAndSettle();

      // Hover scrollbar behavior.
      final TestGesture gesture = await tester.createGesture(kind: ui.PointerDeviceKind.mouse);
      await gesture.addPointer();
      await gesture.moveTo(const Offset(794.0, 5.0));
      await tester.pumpAndSettle();

      expect(
        find.byType(Scrollbar),
        paints
          ..rect(
            rect: const Rect.fromLTRB(792.0, 0.0, 800.0, 600.0),
            color: const Color(0x08000000),
          )
          ..line(
            p1: const Offset(792.0, 0.0),
            p2: const Offset(792.0, 600.0),
            strokeWidth: 1.0,
            color: const Color(0x1a000000),
          )
          ..rrect(
            rrect: RRect.fromRectAndRadius(
              const Rect.fromLTRB(800 - thickness - edgeMargin, 10.0, 798.0, 100.0),
              const Radius.circular(3.0),
            ),
            // Hover color
            color: const Color(0x80000000),
          ),
      );

      scrollController.dispose();
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.linux,
      TargetPlatform.macOS,
      TargetPlatform.windows,
      TargetPlatform.fuchsia,
    }),
  );

  testWidgets(
    'ThemeData colorScheme is used when no ScrollbarTheme is set',
    (WidgetTester tester) async {
      (ScrollController, Widget) buildFrame(ThemeData appTheme) {
        final ScrollController scrollController = ScrollController();
        final ThemeData theme = appTheme.copyWith(
          scrollbarTheme: ScrollbarThemeData(
            trackVisibility: MaterialStateProperty.resolveWith((Set<MaterialState> states) {
              if (states.contains(MaterialState.hovered)) {
                return true;
              }
              return false;
            }),
          ),
        );
        return (
          scrollController,
          MaterialApp(
            theme: theme,
            home: ScrollConfiguration(
              behavior: const NoScrollbarBehavior(),
              child: Scrollbar(
                thumbVisibility: true,
                controller: scrollController,
                child: SingleChildScrollView(
                  controller: scrollController,
                  child: const SizedBox(width: 4000.0, height: 4000.0),
                ),
              ),
            ),
          ),
        );
      }

      // Scrollbar defaults for light themes:
      // - coloring based on ColorScheme.onSurface
      final (ScrollController controller1, Widget frame1) = buildFrame(
        ThemeData(colorScheme: const ColorScheme.light()),
      );
      await tester.pumpWidget(frame1);
      await tester.pumpAndSettle();
      // Idle scrollbar behavior
      expect(
        find.byType(Scrollbar),
        paints..rrect(
          rrect: RRect.fromRectAndRadius(_kMaterialDesignInitialThumbRect, _kDefaultThumbRadius),
          color: _kDefaultIdleThumbColor,
        ),
      );

      // Drag scrollbar behavior
      const double scrollAmount = 10.0;
      TestGesture dragScrollbarGesture = await tester.startGesture(const Offset(797.0, 45.0));
      await tester.pumpAndSettle();

      expect(
        find.byType(Scrollbar),
        paints..rrect(
          rrect: RRect.fromRectAndRadius(_kMaterialDesignInitialThumbRect, _kDefaultThumbRadius),
          // Drag color
          color: _kDefaultDragThumbColor,
        ),
      );

      await dragScrollbarGesture.moveBy(const Offset(0.0, scrollAmount));
      await tester.pumpAndSettle();
      await dragScrollbarGesture.up();
      await tester.pumpAndSettle();

      // Hover scrollbar behavior
      final TestGesture hoverGesture = await tester.createGesture(kind: ui.PointerDeviceKind.mouse);
      await hoverGesture.addPointer();
      addTearDown(hoverGesture.removePointer);
      await hoverGesture.moveTo(const Offset(794.0, 5.0));
      await tester.pumpAndSettle();

      expect(
        find.byType(Scrollbar),
        paints
          ..rect(
            rect: const Rect.fromLTRB(784.0, 0.0, 800.0, 600.0),
            color: const Color(0x08000000),
          )
          ..line(
            p1: const Offset(784.0, 0.0),
            p2: const Offset(784.0, 600.0),
            strokeWidth: 1.0,
            color: const Color(0x1a000000),
          )
          ..rrect(
            rrect: RRect.fromRectAndRadius(
              // Scrollbar thumb is larger
              const Rect.fromLTRB(786.0, 10.0, 798.0, 100.0),
              _kDefaultThumbRadius,
            ),
            // Hover color
            color: const Color(0x80000000),
          ),
      );

      await hoverGesture.moveTo(Offset.zero);

      // Scrollbar defaults for dark themes:
      // - coloring slightly different based on ColorScheme.onSurface
      final (ScrollController controller2, Widget frame2) = buildFrame(
        ThemeData(colorScheme: const ColorScheme.dark()),
      );
      await tester.pumpWidget(frame2);
      await tester.pumpAndSettle(); // Theme change animation

      // Idle scrollbar behavior
      expect(
        find.byType(Scrollbar),
        paints..rrect(
          rrect: RRect.fromRectAndRadius(
            const Rect.fromLTRB(790.0, 10.0, 798.0, 100.0),
            _kDefaultThumbRadius,
          ),
          color: const Color(0x4dffffff),
        ),
      );

      // Drag scrollbar behavior
      dragScrollbarGesture = await tester.startGesture(const Offset(797.0, 45.0));
      await tester.pumpAndSettle();

      expect(
        find.byType(Scrollbar),
        paints..rrect(
          rrect: RRect.fromRectAndRadius(
            const Rect.fromLTRB(790.0, 10.0, 798.0, 100.0),
            _kDefaultThumbRadius,
          ),
          // Drag color
          color: const Color(0xbfffffff),
        ),
      );

      await dragScrollbarGesture.moveBy(const Offset(0.0, scrollAmount));
      await tester.pumpAndSettle();
      await dragScrollbarGesture.up();
      await tester.pumpAndSettle();

      // Hover scrollbar behavior
      await hoverGesture.moveTo(const Offset(794.0, 5.0));
      await tester.pumpAndSettle();

      expect(
        find.byType(Scrollbar),
        paints
          ..rect(
            rect: const Rect.fromLTRB(784.0, 0.0, 800.0, 600.0),
            color: const Color(0x0dffffff),
          )
          ..line(
            p1: const Offset(784.0, 0.0),
            p2: const Offset(784.0, 600.0),
            strokeWidth: 1.0,
            color: const Color(0x40ffffff),
          )
          ..rrect(
            rrect: RRect.fromRectAndRadius(
              // Scrollbar thumb is larger
              const Rect.fromLTRB(786.0, 20.0, 798.0, 110.0),
              _kDefaultThumbRadius,
            ),
            // Hover color
            color: const Color(0xa6ffffff),
          ),
      );

      controller1.dispose();
      controller2.dispose();
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.linux,
      TargetPlatform.macOS,
      TargetPlatform.windows,
      TargetPlatform.fuchsia,
    }),
  );

  testWidgets(
    'ScrollbarThemeData.trackVisibility test',
    (WidgetTester tester) async {
      final ScrollController scrollController = ScrollController();
      bool? getTrackVisibility(Set<MaterialState> states) {
        return true;
      }

      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(useMaterial3: false).copyWith(
            scrollbarTheme: _scrollbarTheme(
              trackVisibility: MaterialStateProperty.resolveWith(getTrackVisibility),
            ),
          ),
          home: ScrollConfiguration(
            behavior: const NoScrollbarBehavior(),
            child: Scrollbar(
              thumbVisibility: true,
              controller: scrollController,
              child: SingleChildScrollView(
                controller: scrollController,
                child: const SizedBox(width: 4000.0, height: 4000.0),
              ),
            ),
          ),
        ),
      );
      await tester.pumpAndSettle();

      expect(
        find.byType(Scrollbar),
        paints
          ..rect(color: const Color(0x08000000))
          ..line(strokeWidth: 1.0, color: const Color(0x1a000000))
          ..rrect(color: const Color(0xff4caf50)),
      );

      scrollController.dispose();
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.linux,
      TargetPlatform.macOS,
      TargetPlatform.windows,
      TargetPlatform.fuchsia,
    }),
  );

  testWidgets('Default ScrollbarTheme debugFillProperties', (WidgetTester tester) async {
    final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();
    const ScrollbarThemeData().debugFillProperties(builder);

    final List<String> description =
        builder.properties
            .where((DiagnosticsNode node) => !node.isFiltered(DiagnosticLevel.info))
            .map((DiagnosticsNode node) => node.toString())
            .toList();

    expect(description, <String>[]);
  });

  testWidgets('ScrollbarTheme implements debugFillProperties', (WidgetTester tester) async {
    final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();
    ScrollbarThemeData(
      thickness: MaterialStateProperty.resolveWith(_getThickness),
      thumbVisibility: MaterialStateProperty.resolveWith(_getThumbVisibility),
      radius: const Radius.circular(3.0),
      thumbColor: MaterialStateProperty.resolveWith(_getThumbColor),
      trackColor: MaterialStateProperty.resolveWith(_getTrackColor),
      trackBorderColor: MaterialStateProperty.resolveWith(_getTrackBorderColor),
      crossAxisMargin: 3.0,
      mainAxisMargin: 6.0,
      minThumbLength: 120.0,
    ).debugFillProperties(builder);

    final List<String> description =
        builder.properties
            .where((DiagnosticsNode node) => !node.isFiltered(DiagnosticLevel.info))
            .map((DiagnosticsNode node) => node.toString())
            .toList();

    expect(description, <String>[
      "thumbVisibility: Instance of '_WidgetStatePropertyWith<bool?>'",
      "thickness: Instance of '_WidgetStatePropertyWith<double?>'",
      'radius: Radius.circular(3.0)',
      "thumbColor: Instance of '_WidgetStatePropertyWith<Color?>'",
      "trackColor: Instance of '_WidgetStatePropertyWith<Color?>'",
      "trackBorderColor: Instance of '_WidgetStatePropertyWith<Color?>'",
      'crossAxisMargin: 3.0',
      'mainAxisMargin: 6.0',
      'minThumbLength: 120.0',
    ]);

    // On the web, Dart doubles and ints are backed by the same kind of object because
    // JavaScript does not support integers. So, the Dart double "4.0" is identical
    // to "4", which results in the web evaluating to the value "4" regardless of which
    // one is used. This results in a difference for doubles in debugFillProperties between
    // the web and the rest of Flutter's target platforms.
  }, skip: kIsWeb); // [intended]
}

class NoScrollbarBehavior extends ScrollBehavior {
  const NoScrollbarBehavior();

  @override
  Widget buildScrollbar(BuildContext context, Widget child, ScrollableDetails details) => child;
}

ScrollbarThemeData _scrollbarTheme({
  MaterialStateProperty<double?>? thickness,
  MaterialStateProperty<bool?>? trackVisibility,
  MaterialStateProperty<bool?>? thumbVisibility,
  Radius radius = const Radius.circular(6.0),
  MaterialStateProperty<Color?>? thumbColor,
  MaterialStateProperty<Color?>? trackColor,
  MaterialStateProperty<Color?>? trackBorderColor,
  double crossAxisMargin = 5.0,
  double mainAxisMargin = 10.0,
  double minThumbLength = 50.0,
}) {
  return ScrollbarThemeData(
    thickness: thickness ?? MaterialStateProperty.resolveWith(_getThickness),
    trackVisibility:
        trackVisibility ??
        MaterialStateProperty.resolveWith((Set<MaterialState> states) {
          if (states.contains(MaterialState.hovered)) {
            return true;
          }
          return false;
        }),
    thumbVisibility: thumbVisibility,
    radius: radius,
    thumbColor: thumbColor ?? MaterialStateProperty.resolveWith(_getThumbColor),
    trackColor: trackColor ?? MaterialStateProperty.resolveWith(_getTrackColor),
    trackBorderColor: trackBorderColor ?? MaterialStateProperty.resolveWith(_getTrackBorderColor),
    crossAxisMargin: crossAxisMargin,
    mainAxisMargin: mainAxisMargin,
    minThumbLength: minThumbLength,
  );
}

double? _getThickness(Set<MaterialState> states) {
  if (states.contains(MaterialState.hovered)) {
    return 20.0;
  }
  return 10.0;
}

bool? _getThumbVisibility(Set<MaterialState> states) => true;

Color? _getThumbColor(Set<MaterialState> states) {
  if (states.contains(MaterialState.dragged)) {
    return Colors.red;
  }
  if (states.contains(MaterialState.hovered)) {
    return Colors.blue;
  }
  return Colors.green;
}

Color? _getTrackColor(Set<MaterialState> states) {
  if (states.contains(MaterialState.hovered)) {
    return Colors.black;
  }
  return null;
}

Color? _getTrackBorderColor(Set<MaterialState> states) {
  if (states.contains(MaterialState.hovered)) {
    return Colors.yellow;
  }
  return null;
}
