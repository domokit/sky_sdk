// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// no-shuffle:
//   //TODO(gspencergoog): Remove this tag once this test's state leaks/test
//   dependencies have been fixed.
//   https://github.com/flutter/flutter/issues/85160
//   Fails with "flutter test --test-randomize-ordering-seed=456"
// reduced-test-set:
//   This file is run as part of a reduced test set in CI on Mac and Windows
//   machines.
@Tags(<String>['reduced-test-set', 'no-shuffle'])
library;

import 'package:flutter/cupertino.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  final ThemeData theme = ThemeData();

  // The "can be constructed" tests that follow are primarily to ensure that any
  // animations started by the progress indicators are stopped at dispose() time.

  testWidgets(
    'LinearProgressIndicator(value: 0.0) can be constructed and has empty semantics by default',
    (WidgetTester tester) async {
      final SemanticsHandle handle = tester.ensureSemantics();
      await tester.pumpWidget(
        Theme(
          data: theme,
          child: const Directionality(
            textDirection: TextDirection.ltr,
            child: Center(
              child: SizedBox(width: 200.0, child: LinearProgressIndicator(value: 0.0)),
            ),
          ),
        ),
      );

      expect(tester.getSemantics(find.byType(LinearProgressIndicator)), matchesSemantics());
      handle.dispose();
    },
  );

  testWidgets(
    'LinearProgressIndicator(value: null) can be constructed and has empty semantics by default',
    (WidgetTester tester) async {
      final SemanticsHandle handle = tester.ensureSemantics();
      await tester.pumpWidget(
        Theme(
          data: theme,
          child: const Directionality(
            textDirection: TextDirection.rtl,
            child: Center(child: SizedBox(width: 200.0, child: LinearProgressIndicator())),
          ),
        ),
      );

      expect(tester.getSemantics(find.byType(LinearProgressIndicator)), matchesSemantics());
      handle.dispose();
    },
  );

  testWidgets('LinearProgressIndicator custom minHeight', (WidgetTester tester) async {
    await tester.pumpWidget(
      Theme(
        data: theme,
        child: const Directionality(
          textDirection: TextDirection.ltr,
          child: Center(
            child: SizedBox(
              width: 200.0,
              child: LinearProgressIndicator(value: 0.25, minHeight: 2.0),
            ),
          ),
        ),
      ),
    );
    expect(
      find.byType(LinearProgressIndicator),
      paints
        ..rect(rect: const Rect.fromLTRB(0.0, 0.0, 200.0, 2.0))
        ..rect(rect: const Rect.fromLTRB(0.0, 0.0, 50.0, 2.0)),
    );

    // Same test, but using the theme
    await tester.pumpWidget(
      Theme(
        data: theme.copyWith(
          progressIndicatorTheme: const ProgressIndicatorThemeData(linearMinHeight: 2.0),
        ),
        child: const Directionality(
          textDirection: TextDirection.ltr,
          child: Center(child: SizedBox(width: 200.0, child: LinearProgressIndicator(value: 0.25))),
        ),
      ),
    );
    expect(
      find.byType(LinearProgressIndicator),
      paints
        ..rect(rect: const Rect.fromLTRB(0.0, 0.0, 200.0, 2.0))
        ..rect(rect: const Rect.fromLTRB(0.0, 0.0, 50.0, 2.0)),
    );
  });

  testWidgets('LinearProgressIndicator paint (LTR)', (WidgetTester tester) async {
    await tester.pumpWidget(
      Theme(
        data: theme,
        child: const Directionality(
          textDirection: TextDirection.ltr,
          child: Center(child: SizedBox(width: 200.0, child: LinearProgressIndicator(value: 0.25))),
        ),
      ),
    );

    expect(
      find.byType(LinearProgressIndicator),
      paints
        ..rect(rect: const Rect.fromLTRB(0.0, 0.0, 200.0, 4.0))
        ..rect(rect: const Rect.fromLTRB(0.0, 0.0, 50.0, 4.0)),
    );

    expect(tester.binding.transientCallbackCount, 0);
  });

  testWidgets('LinearProgressIndicator paint (RTL)', (WidgetTester tester) async {
    await tester.pumpWidget(
      Theme(
        data: theme,
        child: const Directionality(
          textDirection: TextDirection.rtl,
          child: Center(child: SizedBox(width: 200.0, child: LinearProgressIndicator(value: 0.25))),
        ),
      ),
    );

    expect(
      find.byType(LinearProgressIndicator),
      paints
        ..rect(rect: const Rect.fromLTRB(0.0, 0.0, 200.0, 4.0))
        ..rect(rect: const Rect.fromLTRB(150.0, 0.0, 200.0, 4.0)),
    );

    expect(tester.binding.transientCallbackCount, 0);
  });

  testWidgets('LinearProgressIndicator indeterminate (LTR)', (WidgetTester tester) async {
    await tester.pumpWidget(
      Theme(
        data: theme,
        child: const Directionality(
          textDirection: TextDirection.ltr,
          child: Center(child: SizedBox(width: 200.0, child: LinearProgressIndicator())),
        ),
      ),
    );

    await tester.pump(const Duration(milliseconds: 300));
    final double animationValue = const Interval(
      0.0,
      750.0 / 1800.0,
      curve: Cubic(0.2, 0.0, 0.8, 1.0),
    ).transform(300.0 / 1800.0);

    expect(
      find.byType(LinearProgressIndicator),
      paints
        ..rect(rect: const Rect.fromLTRB(0.0, 0.0, 200.0, 4.0))
        ..rect(rect: Rect.fromLTRB(0.0, 0.0, animationValue * 200.0, 4.0)),
    );

    expect(tester.binding.transientCallbackCount, 1);
  });

  testWidgets('LinearProgressIndicator paint (RTL)', (WidgetTester tester) async {
    await tester.pumpWidget(
      Theme(
        data: theme,
        child: const Directionality(
          textDirection: TextDirection.rtl,
          child: Center(child: SizedBox(width: 200.0, child: LinearProgressIndicator())),
        ),
      ),
    );

    await tester.pump(const Duration(milliseconds: 300));
    final double animationValue = const Interval(
      0.0,
      750.0 / 1800.0,
      curve: Cubic(0.2, 0.0, 0.8, 1.0),
    ).transform(300.0 / 1800.0);

    expect(
      find.byType(LinearProgressIndicator),
      paints
        ..rect(rect: const Rect.fromLTRB(0.0, 0.0, 200.0, 4.0))
        ..rect(rect: Rect.fromLTRB(200.0 - animationValue * 200.0, 0.0, 200.0, 4.0)),
    );

    expect(tester.binding.transientCallbackCount, 1);
  });

  testWidgets('LinearProgressIndicator with colors', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(
        home: Center(child: SizedBox(width: 200.0, child: LinearProgressIndicator(value: 0.25))),
      ),
    );

    // Defaults.
    expect(
      find.byType(LinearProgressIndicator),
      paints
        ..rect(
          rect: const Rect.fromLTRB(0.0, 0.0, 200.0, 4.0),
          color: theme.colorScheme.secondaryContainer,
        )
        ..rect(rect: const Rect.fromLTRB(0.0, 0.0, 50.0, 4.0), color: theme.colorScheme.primary),
    );

    // With valueColor & color provided
    await tester.pumpWidget(
      Theme(
        data: theme,
        child: const Directionality(
          textDirection: TextDirection.ltr,
          child: Center(
            child: SizedBox(
              width: 200.0,
              child: LinearProgressIndicator(
                value: 0.25,
                backgroundColor: Colors.black,
                color: Colors.blue,
                valueColor: AlwaysStoppedAnimation<Color>(Colors.white),
              ),
            ),
          ),
        ),
      ),
    );

    // Should use valueColor
    expect(
      find.byType(LinearProgressIndicator),
      paints
        ..rect(rect: const Rect.fromLTRB(0.0, 0.0, 200.0, 4.0))
        ..rect(rect: const Rect.fromLTRB(0.0, 0.0, 50.0, 4.0), color: Colors.white),
    );

    // With just color provided
    await tester.pumpWidget(
      Theme(
        data: theme,
        child: const Directionality(
          textDirection: TextDirection.ltr,
          child: Center(
            child: SizedBox(
              width: 200.0,
              child: LinearProgressIndicator(
                value: 0.25,
                backgroundColor: Colors.black,
                color: Colors.white12,
              ),
            ),
          ),
        ),
      ),
    );

    // Should use color
    expect(
      find.byType(LinearProgressIndicator),
      paints
        ..rect(rect: const Rect.fromLTRB(0.0, 0.0, 200.0, 4.0))
        ..rect(rect: const Rect.fromLTRB(0.0, 0.0, 50.0, 4.0), color: Colors.white12),
    );

    // With no color provided
    const Color primaryColor = Color(0xff008800);
    await tester.pumpWidget(
      Theme(
        data: theme.copyWith(colorScheme: ColorScheme.fromSwatch().copyWith(primary: primaryColor)),
        child: const Directionality(
          textDirection: TextDirection.ltr,
          child: Center(
            child: SizedBox(
              width: 200.0,
              child: LinearProgressIndicator(value: 0.25, backgroundColor: Colors.black),
            ),
          ),
        ),
      ),
    );

    // Should use the theme's primary color
    expect(
      find.byType(LinearProgressIndicator),
      paints
        ..rect(rect: const Rect.fromLTRB(0.0, 0.0, 200.0, 4.0))
        ..rect(rect: const Rect.fromLTRB(0.0, 0.0, 50.0, 4.0), color: primaryColor),
    );

    // With ProgressIndicatorTheme colors
    const Color indicatorColor = Color(0xff0000ff);
    await tester.pumpWidget(
      Theme(
        data: theme.copyWith(
          progressIndicatorTheme: const ProgressIndicatorThemeData(
            color: indicatorColor,
            linearTrackColor: Colors.black,
          ),
        ),
        child: const Directionality(
          textDirection: TextDirection.ltr,
          child: Center(child: SizedBox(width: 200.0, child: LinearProgressIndicator(value: 0.25))),
        ),
      ),
    );

    // Should use the progress indicator theme colors
    expect(
      find.byType(LinearProgressIndicator),
      paints
        ..rect(rect: const Rect.fromLTRB(0.0, 0.0, 200.0, 4.0))
        ..rect(rect: const Rect.fromLTRB(0.0, 0.0, 50.0, 4.0), color: indicatorColor),
    );
  });

  testWidgets('LinearProgressIndicator with animation with null colors', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      Theme(
        data: theme,
        child: const Directionality(
          textDirection: TextDirection.ltr,
          child: Center(
            child: SizedBox(
              width: 200.0,
              child: LinearProgressIndicator(
                value: 0.25,
                valueColor: AlwaysStoppedAnimation<Color?>(null),
                backgroundColor: Colors.black,
              ),
            ),
          ),
        ),
      ),
    );

    expect(
      find.byType(LinearProgressIndicator),
      paints
        ..rect(rect: const Rect.fromLTRB(0.0, 0.0, 200.0, 4.0))
        ..rect(rect: const Rect.fromLTRB(0.0, 0.0, 50.0, 4.0)),
    );
  });

  testWidgets(
    'CircularProgressIndicator(value: 0.0) can be constructed and has value semantics by default',
    (WidgetTester tester) async {
      final SemanticsHandle handle = tester.ensureSemantics();
      await tester.pumpWidget(
        Theme(
          data: theme,
          child: const Directionality(
            textDirection: TextDirection.ltr,
            child: Center(child: CircularProgressIndicator(value: 0.0)),
          ),
        ),
      );

      expect(
        tester.getSemantics(find.byType(CircularProgressIndicator)),
        matchesSemantics(value: '0%', textDirection: TextDirection.ltr),
      );
      handle.dispose();
    },
  );

  testWidgets(
    'CircularProgressIndicator(value: null) can be constructed and has empty semantics by default',
    (WidgetTester tester) async {
      final SemanticsHandle handle = tester.ensureSemantics();
      await tester.pumpWidget(
        Theme(data: theme, child: const Center(child: CircularProgressIndicator())),
      );

      expect(tester.getSemantics(find.byType(CircularProgressIndicator)), matchesSemantics());
      handle.dispose();
    },
  );

  testWidgets('LinearProgressIndicator causes a repaint when it changes', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      Theme(
        data: theme,
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: ListView(children: const <Widget>[LinearProgressIndicator(value: 0.0)]),
        ),
      ),
    );
    final List<Layer> layers1 = tester.layers;
    await tester.pumpWidget(
      Theme(
        data: theme,
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: ListView(children: const <Widget>[LinearProgressIndicator(value: 0.5)]),
        ),
      ),
    );
    final List<Layer> layers2 = tester.layers;
    expect(layers1, isNot(equals(layers2)));
  });

  testWidgets('CircularProgressIndicator stroke width', (WidgetTester tester) async {
    await tester.pumpWidget(Theme(data: theme, child: const CircularProgressIndicator()));

    expect(find.byType(CircularProgressIndicator), paints..arc(strokeWidth: 4.0));

    await tester.pumpWidget(
      Theme(data: theme, child: const CircularProgressIndicator(strokeWidth: 16.0)),
    );

    expect(find.byType(CircularProgressIndicator), paints..arc(strokeWidth: 16.0));
  });

  testWidgets('CircularProgressIndicator strokeAlign', (WidgetTester tester) async {
    await tester.pumpWidget(Theme(data: theme, child: const CircularProgressIndicator()));
    expect(
      find.byType(CircularProgressIndicator),
      paints..arc(rect: Offset.zero & const Size(800.0, 600.0)),
    );

    await tester.pumpWidget(
      Theme(
        data: theme,
        child: const CircularProgressIndicator(
          strokeAlign: CircularProgressIndicator.strokeAlignInside,
        ),
      ),
    );
    expect(
      find.byType(CircularProgressIndicator),
      paints..arc(rect: const Offset(2.0, 2.0) & const Size(796.0, 596.0)),
    );

    await tester.pumpWidget(
      Theme(
        data: theme,
        child: const CircularProgressIndicator(
          strokeAlign: CircularProgressIndicator.strokeAlignOutside,
        ),
      ),
    );
    expect(
      find.byType(CircularProgressIndicator),
      paints..arc(rect: const Offset(-2.0, -2.0) & const Size(804.0, 604.0)),
    );

    // Unbounded alignment.
    await tester.pumpWidget(
      Theme(data: theme, child: const CircularProgressIndicator(strokeAlign: 2.0)),
    );
    expect(
      find.byType(CircularProgressIndicator),
      paints..arc(rect: const Offset(-4.0, -4.0) & const Size(808.0, 608.0)),
    );
  });

  testWidgets('CircularProgressIndicator with strokeCap', (WidgetTester tester) async {
    await tester.pumpWidget(const CircularProgressIndicator());
    expect(
      find.byType(CircularProgressIndicator),
      paints..arc(strokeCap: StrokeCap.square),
      reason: 'Default indeterminate strokeCap is StrokeCap.square.',
    );

    await tester.pumpWidget(
      const Directionality(
        textDirection: TextDirection.ltr,
        child: CircularProgressIndicator(value: 0.5),
      ),
    );
    expect(
      find.byType(CircularProgressIndicator),
      paints..arc(strokeCap: StrokeCap.butt),
      reason: 'Default determinate strokeCap is StrokeCap.butt.',
    );

    await tester.pumpWidget(const CircularProgressIndicator(strokeCap: StrokeCap.butt));
    expect(
      find.byType(CircularProgressIndicator),
      paints..arc(strokeCap: StrokeCap.butt),
      reason: 'strokeCap can be set to StrokeCap.butt, and will not be overridden.',
    );

    await tester.pumpWidget(const CircularProgressIndicator(strokeCap: StrokeCap.round));
    expect(find.byType(CircularProgressIndicator), paints..arc(strokeCap: StrokeCap.round));
  });

  testWidgets('LinearProgressIndicator with indicatorBorderRadius', (WidgetTester tester) async {
    await tester.pumpWidget(
      Theme(
        data: theme,
        child: const Directionality(
          textDirection: TextDirection.ltr,
          child: Center(
            child: SizedBox(
              width: 100.0,
              height: 4.0,
              child: LinearProgressIndicator(
                value: 0.25,
                borderRadius: BorderRadius.all(Radius.circular(10)),
              ),
            ),
          ),
        ),
      ),
    );
    expect(
      find.byType(LinearProgressIndicator),
      paints
        ..rrect(rrect: RRect.fromLTRBR(0.0, 0.0, 100.0, 4.0, const Radius.circular(10.0)))
        ..rrect(
          rrect: RRect.fromRectAndRadius(
            const Rect.fromLTRB(0.0, 0.0, 25.0, 4.0),
            const Radius.circular(10.0),
          ),
        ),
    );
    expect(tester.binding.transientCallbackCount, 0);
  });

  testWidgets('CircularProgressIndicator paint colors', (WidgetTester tester) async {
    const Color green = Color(0xFF00FF00);
    const Color blue = Color(0xFF0000FF);
    const Color red = Color(0xFFFF0000);

    // With valueColor & color provided
    await tester.pumpWidget(
      Theme(
        data: theme,
        child: const CircularProgressIndicator(
          color: red,
          valueColor: AlwaysStoppedAnimation<Color>(blue),
        ),
      ),
    );
    expect(find.byType(CircularProgressIndicator), paintsExactlyCountTimes(#drawArc, 1));
    expect(find.byType(CircularProgressIndicator), paints..arc(color: blue));

    // With just color provided
    await tester.pumpWidget(Theme(data: theme, child: const CircularProgressIndicator(color: red)));
    expect(find.byType(CircularProgressIndicator), paintsExactlyCountTimes(#drawArc, 1));
    expect(find.byType(CircularProgressIndicator), paints..arc(color: red));

    // With no color provided
    await tester.pumpWidget(
      Theme(
        data: theme.copyWith(colorScheme: ColorScheme.fromSwatch().copyWith(primary: green)),
        child: const CircularProgressIndicator(),
      ),
    );
    expect(find.byType(CircularProgressIndicator), paintsExactlyCountTimes(#drawArc, 1));
    expect(find.byType(CircularProgressIndicator), paints..arc(color: green));

    // With background
    await tester.pumpWidget(
      Theme(
        data: theme,
        child: const CircularProgressIndicator(backgroundColor: green, color: blue),
      ),
    );
    expect(find.byType(CircularProgressIndicator), paintsExactlyCountTimes(#drawArc, 2));
    expect(
      find.byType(CircularProgressIndicator),
      paints
        ..arc(color: green)
        ..arc(color: blue),
    );

    // With ProgressIndicatorTheme
    await tester.pumpWidget(
      Theme(
        data: theme.copyWith(
          progressIndicatorTheme: const ProgressIndicatorThemeData(
            color: green,
            circularTrackColor: blue,
          ),
        ),
        child: const CircularProgressIndicator(),
      ),
    );
    expect(find.byType(CircularProgressIndicator), paintsExactlyCountTimes(#drawArc, 2));
    expect(
      find.byType(CircularProgressIndicator),
      paints
        ..arc(color: blue)
        ..arc(color: green),
    );
  });

  testWidgets('RefreshProgressIndicator paint colors', (WidgetTester tester) async {
    const Color green = Color(0xFF00FF00);
    const Color blue = Color(0xFF0000FF);
    const Color red = Color(0xFFFF0000);

    // With valueColor & color provided
    await tester.pumpWidget(
      Theme(
        data: theme,
        child: const RefreshProgressIndicator(
          color: red,
          valueColor: AlwaysStoppedAnimation<Color>(blue),
        ),
      ),
    );
    expect(find.byType(RefreshProgressIndicator), paintsExactlyCountTimes(#drawArc, 1));
    expect(find.byType(RefreshProgressIndicator), paints..arc(color: blue));

    // With just color provided
    await tester.pumpWidget(Theme(data: theme, child: const RefreshProgressIndicator(color: red)));
    expect(find.byType(RefreshProgressIndicator), paintsExactlyCountTimes(#drawArc, 1));
    expect(find.byType(RefreshProgressIndicator), paints..arc(color: red));

    // With no color provided
    await tester.pumpWidget(
      Theme(
        data: theme.copyWith(colorScheme: ColorScheme.fromSwatch().copyWith(primary: green)),
        child: const RefreshProgressIndicator(),
      ),
    );
    expect(find.byType(RefreshProgressIndicator), paintsExactlyCountTimes(#drawArc, 1));
    expect(find.byType(RefreshProgressIndicator), paints..arc(color: green));

    // With background
    await tester.pumpWidget(
      Theme(
        data: theme,
        child: const RefreshProgressIndicator(color: blue, backgroundColor: green),
      ),
    );
    expect(find.byType(RefreshProgressIndicator), paintsExactlyCountTimes(#drawArc, 1));
    expect(find.byType(RefreshProgressIndicator), paints..arc(color: blue));
    final Material backgroundMaterial = tester.widget(
      find.descendant(of: find.byType(RefreshProgressIndicator), matching: find.byType(Material)),
    );
    expect(backgroundMaterial.type, MaterialType.circle);
    expect(backgroundMaterial.color, green);

    // With ProgressIndicatorTheme
    await tester.pumpWidget(
      Theme(
        data: theme.copyWith(
          progressIndicatorTheme: const ProgressIndicatorThemeData(
            color: green,
            refreshBackgroundColor: blue,
          ),
        ),
        child: const RefreshProgressIndicator(),
      ),
    );
    expect(find.byType(RefreshProgressIndicator), paintsExactlyCountTimes(#drawArc, 1));
    expect(find.byType(RefreshProgressIndicator), paints..arc(color: green));
    final Material themeBackgroundMaterial = tester.widget(
      find.descendant(of: find.byType(RefreshProgressIndicator), matching: find.byType(Material)),
    );
    expect(themeBackgroundMaterial.type, MaterialType.circle);
    expect(themeBackgroundMaterial.color, blue);
  });

  testWidgets('RefreshProgressIndicator with a round indicator', (WidgetTester tester) async {
    await tester.pumpWidget(const RefreshProgressIndicator());
    expect(
      find.byType(RefreshProgressIndicator),
      paints..arc(strokeCap: StrokeCap.square),
      reason: 'Default indeterminate strokeCap is StrokeCap.square',
    );

    await tester.pumpWidget(
      Theme(
        data: theme,
        child: const Directionality(
          textDirection: TextDirection.ltr,
          child: Center(
            child: SizedBox(
              width: 200.0,
              child: RefreshProgressIndicator(strokeCap: StrokeCap.round),
            ),
          ),
        ),
      ),
    );
    expect(find.byType(RefreshProgressIndicator), paints..arc(strokeCap: StrokeCap.round));
  });

  testWidgets(
    'Indeterminate RefreshProgressIndicator keeps spinning until end of time (approximate)',
    (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/issues/13782

      await tester.pumpWidget(
        Theme(
          data: theme,
          child: const Directionality(
            textDirection: TextDirection.ltr,
            child: Center(child: SizedBox(width: 200.0, child: RefreshProgressIndicator())),
          ),
        ),
      );
      expect(tester.hasRunningAnimations, isTrue);

      await tester.pump(const Duration(seconds: 5));
      expect(tester.hasRunningAnimations, isTrue);

      await tester.pump(const Duration(milliseconds: 1));
      expect(tester.hasRunningAnimations, isTrue);

      await tester.pump(const Duration(days: 9999));
      expect(tester.hasRunningAnimations, isTrue);
    },
  );

  testWidgets('Material2 - RefreshProgressIndicator uses expected animation', (
    WidgetTester tester,
  ) async {
    final AnimationSheetBuilder animationSheet = AnimationSheetBuilder(
      frameSize: const Size(50, 50),
    );
    addTearDown(animationSheet.dispose);

    await tester.pumpFrames(
      animationSheet.record(
        Theme(data: ThemeData(useMaterial3: false), child: const _RefreshProgressIndicatorGolden()),
      ),
      const Duration(seconds: 3),
    );

    await expectLater(
      animationSheet.collate(20),
      matchesGoldenFile('m2_material.refresh_progress_indicator.png'),
    );
  }, skip: isBrowser); // https://github.com/flutter/flutter/issues/56001

  testWidgets('Material3 - RefreshProgressIndicator uses expected animation', (
    WidgetTester tester,
  ) async {
    final AnimationSheetBuilder animationSheet = AnimationSheetBuilder(
      frameSize: const Size(50, 50),
    );
    addTearDown(animationSheet.dispose);

    await tester.pumpFrames(
      animationSheet.record(
        Theme(data: ThemeData(useMaterial3: true), child: const _RefreshProgressIndicatorGolden()),
      ),
      const Duration(seconds: 3),
    );

    await expectLater(
      animationSheet.collate(20),
      matchesGoldenFile('m3_material.refresh_progress_indicator.png'),
    );
  }, skip: isBrowser); // https://github.com/flutter/flutter/issues/56001

  testWidgets('Determinate CircularProgressIndicator stops the animator', (
    WidgetTester tester,
  ) async {
    double? progressValue;
    late StateSetter setState;
    await tester.pumpWidget(
      Theme(
        data: theme,
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: Center(
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setter) {
                setState = setter;
                return CircularProgressIndicator(value: progressValue);
              },
            ),
          ),
        ),
      ),
    );
    expect(tester.hasRunningAnimations, isTrue);

    setState(() {
      progressValue = 1.0;
    });
    await tester.pump(const Duration(milliseconds: 1));
    expect(tester.hasRunningAnimations, isFalse);

    setState(() {
      progressValue = null;
    });
    await tester.pump(const Duration(milliseconds: 1));
    expect(tester.hasRunningAnimations, isTrue);
  });

  testWidgets('LinearProgressIndicator with height 12.0', (WidgetTester tester) async {
    await tester.pumpWidget(
      Theme(
        data: theme,
        child: const Directionality(
          textDirection: TextDirection.ltr,
          child: Center(
            child: SizedBox(
              width: 100.0,
              height: 12.0,
              child: LinearProgressIndicator(value: 0.25),
            ),
          ),
        ),
      ),
    );
    expect(
      find.byType(LinearProgressIndicator),
      paints
        ..rect(rect: const Rect.fromLTRB(0.0, 0.0, 100.0, 12.0))
        ..rect(rect: const Rect.fromLTRB(0.0, 0.0, 25.0, 12.0)),
    );
    expect(tester.binding.transientCallbackCount, 0);
  });

  testWidgets('LinearProgressIndicator with a height less than the minimum', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      Theme(
        data: theme,
        child: const Directionality(
          textDirection: TextDirection.ltr,
          child: Center(
            child: SizedBox(width: 100.0, height: 3.0, child: LinearProgressIndicator(value: 0.25)),
          ),
        ),
      ),
    );
    expect(
      find.byType(LinearProgressIndicator),
      paints
        ..rect(rect: const Rect.fromLTRB(0.0, 0.0, 100.0, 3.0))
        ..rect(rect: const Rect.fromLTRB(0.0, 0.0, 25.0, 3.0)),
    );
    expect(tester.binding.transientCallbackCount, 0);
  });

  testWidgets('LinearProgressIndicator with default height', (WidgetTester tester) async {
    await tester.pumpWidget(
      Theme(
        data: theme,
        child: const Directionality(
          textDirection: TextDirection.ltr,
          child: Center(
            child: SizedBox(width: 100.0, height: 4.0, child: LinearProgressIndicator(value: 0.25)),
          ),
        ),
      ),
    );
    expect(
      find.byType(LinearProgressIndicator),
      paints
        ..rect(rect: const Rect.fromLTRB(0.0, 0.0, 100.0, 4.0))
        ..rect(rect: const Rect.fromLTRB(0.0, 0.0, 25.0, 4.0)),
    );
    expect(tester.binding.transientCallbackCount, 0);
  });

  testWidgets('LinearProgressIndicator can be made accessible', (WidgetTester tester) async {
    final SemanticsHandle handle = tester.ensureSemantics();
    final GlobalKey key = GlobalKey();
    const String label = 'Label';
    const String value = '25%';
    await tester.pumpWidget(
      Theme(
        data: theme,
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: LinearProgressIndicator(
            key: key,
            value: 0.25,
            semanticsLabel: label,
            semanticsValue: value,
          ),
        ),
      ),
    );

    expect(
      tester.getSemantics(find.byKey(key)),
      matchesSemantics(textDirection: TextDirection.ltr, label: label, value: value),
    );

    handle.dispose();
  });

  testWidgets('LinearProgressIndicator that is determinate gets default a11y value', (
    WidgetTester tester,
  ) async {
    final SemanticsHandle handle = tester.ensureSemantics();
    final GlobalKey key = GlobalKey();
    const String label = 'Label';
    await tester.pumpWidget(
      Theme(
        data: theme,
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: LinearProgressIndicator(key: key, value: 0.25, semanticsLabel: label),
        ),
      ),
    );

    expect(
      tester.getSemantics(find.byKey(key)),
      matchesSemantics(textDirection: TextDirection.ltr, label: label, value: '25%'),
    );

    handle.dispose();
  });

  testWidgets(
    'LinearProgressIndicator that is determinate does not default a11y value when label is null',
    (WidgetTester tester) async {
      final SemanticsHandle handle = tester.ensureSemantics();
      final GlobalKey key = GlobalKey();
      await tester.pumpWidget(
        Theme(
          data: theme,
          child: Directionality(
            textDirection: TextDirection.ltr,
            child: LinearProgressIndicator(key: key, value: 0.25),
          ),
        ),
      );

      expect(tester.getSemantics(find.byKey(key)), matchesSemantics());

      handle.dispose();
    },
  );

  testWidgets('LinearProgressIndicator that is indeterminate does not default a11y value', (
    WidgetTester tester,
  ) async {
    final SemanticsHandle handle = tester.ensureSemantics();
    final GlobalKey key = GlobalKey();
    const String label = 'Progress';
    await tester.pumpWidget(
      Theme(
        data: theme,
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: LinearProgressIndicator(key: key, value: 0.25, semanticsLabel: label),
        ),
      ),
    );

    expect(
      tester.getSemantics(find.byKey(key)),
      matchesSemantics(textDirection: TextDirection.ltr, label: label),
    );

    handle.dispose();
  });

  testWidgets('CircularProgressIndicator can be made accessible', (WidgetTester tester) async {
    final SemanticsHandle handle = tester.ensureSemantics();
    final GlobalKey key = GlobalKey();
    const String label = 'Label';
    const String value = '25%';
    await tester.pumpWidget(
      Theme(
        data: theme,
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: CircularProgressIndicator(
            key: key,
            value: 0.25,
            semanticsLabel: label,
            semanticsValue: value,
          ),
        ),
      ),
    );

    expect(
      tester.getSemantics(find.byKey(key)),
      matchesSemantics(textDirection: TextDirection.ltr, label: label, value: value),
    );

    handle.dispose();
  });

  testWidgets('RefreshProgressIndicator can be made accessible', (WidgetTester tester) async {
    final SemanticsHandle handle = tester.ensureSemantics();
    final GlobalKey key = GlobalKey();
    const String label = 'Label';
    const String value = '25%';
    await tester.pumpWidget(
      Theme(
        data: theme,
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: RefreshProgressIndicator(key: key, semanticsLabel: label, semanticsValue: value),
        ),
      ),
    );

    expect(
      tester.getSemantics(find.byKey(key)),
      matchesSemantics(textDirection: TextDirection.ltr, label: label, value: value),
    );

    handle.dispose();
  });

  testWidgets('Material2 - Indeterminate CircularProgressIndicator uses expected animation', (
    WidgetTester tester,
  ) async {
    final AnimationSheetBuilder animationSheet = AnimationSheetBuilder(
      frameSize: const Size(40, 40),
    );
    addTearDown(animationSheet.dispose);

    await tester.pumpFrames(
      animationSheet.record(
        Theme(
          data: ThemeData(useMaterial3: false),
          child: const Directionality(
            textDirection: TextDirection.ltr,
            child: Padding(padding: EdgeInsets.all(4), child: CircularProgressIndicator()),
          ),
        ),
      ),
      const Duration(seconds: 2),
    );

    await expectLater(
      animationSheet.collate(20),
      matchesGoldenFile('m2_material.circular_progress_indicator.indeterminate.png'),
    );
  }, skip: isBrowser); // https://github.com/flutter/flutter/issues/56001

  testWidgets('Material3 - Indeterminate CircularProgressIndicator uses expected animation', (
    WidgetTester tester,
  ) async {
    final AnimationSheetBuilder animationSheet = AnimationSheetBuilder(
      frameSize: const Size(40, 40),
    );
    addTearDown(animationSheet.dispose);

    await tester.pumpFrames(
      animationSheet.record(
        Theme(
          data: ThemeData(useMaterial3: true),
          child: const Directionality(
            textDirection: TextDirection.ltr,
            child: Padding(padding: EdgeInsets.all(4), child: CircularProgressIndicator()),
          ),
        ),
      ),
      const Duration(seconds: 2),
    );

    await expectLater(
      animationSheet.collate(20),
      matchesGoldenFile('m3_material.circular_progress_indicator.indeterminate.png'),
    );
  }, skip: isBrowser); // https://github.com/flutter/flutter/issues/56001

  testWidgets(
    'Adaptive CircularProgressIndicator displays CupertinoActivityIndicator in iOS',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(),
          home: const Scaffold(body: Material(child: CircularProgressIndicator.adaptive())),
        ),
      );

      expect(find.byType(CupertinoActivityIndicator), findsOneWidget);
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  testWidgets(
    'Adaptive CircularProgressIndicator displays CupertinoActivityIndicator in iOS/macOS',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(),
          home: const Scaffold(
            body: Material(child: CircularProgressIndicator.adaptive(value: 0.5)),
          ),
        ),
      );

      expect(find.byType(CupertinoActivityIndicator), findsOneWidget);
      final double actualProgress =
          tester
              .widget<CupertinoActivityIndicator>(find.byType(CupertinoActivityIndicator))
              .progress;
      expect(actualProgress, 0.5);
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  testWidgets(
    'Adaptive CircularProgressIndicator can use backgroundColor to change tick color for iOS',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(),
          home: const Scaffold(
            body: Material(
              child: CircularProgressIndicator.adaptive(backgroundColor: Color(0xFF5D3FD3)),
            ),
          ),
        ),
      );

      expect(
        find.byType(CupertinoActivityIndicator),
        paints..rrect(
          rrect: const RRect.fromLTRBXY(-1, -10 / 3, 1, -10, 1, 1),
          color: const Color(0x935D3FD3),
        ),
      );
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  testWidgets(
    'Adaptive CircularProgressIndicator does not display CupertinoActivityIndicator in non-iOS',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          theme: theme,
          home: const Scaffold(body: Material(child: CircularProgressIndicator.adaptive())),
        ),
      );

      expect(find.byType(CupertinoActivityIndicator), findsNothing);
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.android,
      TargetPlatform.fuchsia,
      TargetPlatform.windows,
      TargetPlatform.linux,
    }),
  );

  testWidgets('ProgressIndicatorTheme.wrap() always creates a new ProgressIndicatorTheme', (
    WidgetTester tester,
  ) async {
    late BuildContext builderContext;

    const ProgressIndicatorThemeData themeData = ProgressIndicatorThemeData(
      color: Color(0xFFFF0000),
      linearTrackColor: Color(0xFF00FF00),
    );

    final ProgressIndicatorTheme progressTheme = ProgressIndicatorTheme(
      data: themeData,
      child: Builder(
        builder: (BuildContext context) {
          builderContext = context;
          return const LinearProgressIndicator(value: 0.5);
        },
      ),
    );

    await tester.pumpWidget(MaterialApp(home: Theme(data: theme, child: progressTheme)));
    final Widget wrappedTheme = progressTheme.wrap(builderContext, Container());

    // Make sure the returned widget is a new ProgressIndicatorTheme instance
    // with the same theme data as the original.
    expect(wrappedTheme, isNot(equals(progressTheme)));
    expect(wrappedTheme, isInstanceOf<ProgressIndicatorTheme>());
    expect((wrappedTheme as ProgressIndicatorTheme).data, themeData);
  });

  testWidgets('Material3 - Default size of CircularProgressIndicator', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(home: Scaffold(body: Material(child: CircularProgressIndicator()))),
    );

    expect(tester.getSize(find.byType(CircularProgressIndicator)), const Size(36, 36));
  });

  testWidgets('Material3 - Default size of CircularProgressIndicator when year2023 is false', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      const MaterialApp(
        home: Scaffold(body: Material(child: CircularProgressIndicator(year2023: false))),
      ),
    );

    expect(tester.getSize(find.byType(CircularProgressIndicator)), const Size(48, 48));
  });

  testWidgets('RefreshProgressIndicator using fields correctly', (WidgetTester tester) async {
    Future<void> pumpIndicator(RefreshProgressIndicator indicator) {
      return tester.pumpWidget(Theme(data: theme, child: indicator));
    }

    // With default values.
    await pumpIndicator(const RefreshProgressIndicator());
    Material material = tester.widget(
      find.descendant(of: find.byType(RefreshProgressIndicator), matching: find.byType(Material)),
    );
    Padding padding = tester.widget(
      find
          .descendant(of: find.byType(RefreshProgressIndicator), matching: find.byType(Padding))
          .first,
    );
    Padding innerPadding = tester.widget(
      find
          .descendant(
            of: find.descendant(
              of: find.byType(RefreshProgressIndicator),
              matching: find.byType(Material),
            ),
            matching: find.byType(Padding),
          )
          .last,
    );
    expect(material.elevation, 2.0);
    expect(padding.padding, const EdgeInsets.all(4.0));
    expect(innerPadding.padding, const EdgeInsets.all(12.0));

    // With values provided.
    const double testElevation = 1.0;
    const EdgeInsetsGeometry testIndicatorMargin = EdgeInsets.all(6.0);
    const EdgeInsetsGeometry testIndicatorPadding = EdgeInsets.all(10.0);
    await pumpIndicator(
      const RefreshProgressIndicator(
        elevation: testElevation,
        indicatorMargin: testIndicatorMargin,
        indicatorPadding: testIndicatorPadding,
      ),
    );
    material = tester.widget(
      find.descendant(of: find.byType(RefreshProgressIndicator), matching: find.byType(Material)),
    );
    padding = tester.widget(
      find
          .descendant(of: find.byType(RefreshProgressIndicator), matching: find.byType(Padding))
          .first,
    );
    innerPadding = tester.widget(
      find
          .descendant(
            of: find.descendant(
              of: find.byType(RefreshProgressIndicator),
              matching: find.byType(Material),
            ),
            matching: find.byType(Padding),
          )
          .last,
    );
    expect(material.elevation, testElevation);
    expect(padding.padding, testIndicatorMargin);
    expect(innerPadding.padding, testIndicatorPadding);
  });

  testWidgets('LinearProgressIndicator default stop indicator when year2023 is false', (
    WidgetTester tester,
  ) async {
    Widget buildIndicator({required TextDirection textDirection}) {
      return Directionality(
        textDirection: textDirection,
        child: const Center(
          child: SizedBox(
            width: 200.0,
            child: LinearProgressIndicator(year2023: false, value: 0.5),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildIndicator(textDirection: TextDirection.ltr));
    expect(
      find.byType(LinearProgressIndicator),
      paints..circle(x: 198.0, y: 2.0, radius: 2.0, color: theme.colorScheme.primary),
    );

    await tester.pumpWidget(buildIndicator(textDirection: TextDirection.rtl));
    expect(
      find.byType(LinearProgressIndicator),
      paints..circle(x: 2.0, y: 2.0, radius: 2.0, color: theme.colorScheme.primary),
    );
  });

  testWidgets('Indeterminate LinearProgressIndicator does not paint stop indicator', (
    WidgetTester tester,
  ) async {
    Widget buildIndicator({double? value}) {
      return Directionality(
        textDirection: TextDirection.ltr,
        child: Center(
          child: SizedBox(
            width: 200.0,
            child: LinearProgressIndicator(year2023: false, value: value),
          ),
        ),
      );
    }

    // Determinate LinearProgressIndicator paints stop indicator.
    await tester.pumpWidget(buildIndicator(value: 0.5));
    expect(
      find.byType(LinearProgressIndicator),
      // Stop indicator.
      paints..circle(x: 198.0, y: 2.0, radius: 2.0, color: theme.colorScheme.primary),
    );

    // Indeterminate LinearProgressIndicator does not paint stop indicator.
    await tester.pumpWidget(buildIndicator());
    expect(
      find.byType(LinearProgressIndicator),
      // Stop indicator.
      isNot(paints..circle(x: 198.0, y: 2.0, radius: 2.0, color: theme.colorScheme.primary)),
    );
  });

  testWidgets('Can customise LinearProgressIndicator stop indicator when year2023 is false', (
    WidgetTester tester,
  ) async {
    const Color stopIndicatorColor = Color(0XFF00FF00);
    const double stopIndicatorRadius = 5.0;
    Widget buildIndicator({Color? stopIndicatorColor, double? stopIndicatorRadius}) {
      return Directionality(
        textDirection: TextDirection.ltr,
        child: Center(
          child: SizedBox(
            width: 200.0,
            child: LinearProgressIndicator(
              year2023: false,
              stopIndicatorColor: stopIndicatorColor,
              stopIndicatorRadius: stopIndicatorRadius,
              minHeight: 20.0,
              value: 0.5,
            ),
          ),
        ),
      );
    }

    // Test customized stop indicator.
    await tester.pumpWidget(
      buildIndicator(
        stopIndicatorColor: stopIndicatorColor,
        stopIndicatorRadius: stopIndicatorRadius,
      ),
    );
    expect(
      find.byType(LinearProgressIndicator),
      // Stop indicator.
      paints..circle(x: 190.0, y: 10.0, radius: stopIndicatorRadius, color: stopIndicatorColor),
    );

    // Remove stop indicator.
    await tester.pumpWidget(buildIndicator(stopIndicatorRadius: 0));
    expect(
      find.byType(LinearProgressIndicator),
      // Stop indicator.
      isNot(paints..circle(color: stopIndicatorColor)),
    );

    // Test stop indicator with transparent color.
    await tester.pumpWidget(buildIndicator(stopIndicatorColor: const Color(0x00000000)));
    expect(
      find.byType(LinearProgressIndicator),
      // Stop indicator.
      paints..circle(color: const Color(0x00000000)),
    );
  });

  testWidgets('Stop indicator size cannot be larger than the progress indicator', (
    WidgetTester tester,
  ) async {
    Widget buildIndicator({double? stopIndicatorRadius, double? minHeight}) {
      return Directionality(
        textDirection: TextDirection.ltr,
        child: Center(
          child: SizedBox(
            width: 200.0,
            child: LinearProgressIndicator(
              year2023: false,
              stopIndicatorRadius: stopIndicatorRadius,
              minHeight: minHeight,
              value: 0.5,
            ),
          ),
        ),
      );
    }

    // Test stop indicator radius equals to minHeight.
    await tester.pumpWidget(buildIndicator(stopIndicatorRadius: 10.0, minHeight: 20.0));
    expect(
      find.byType(LinearProgressIndicator),
      paints..circle(x: 190.0, y: 10.0, radius: 10.0, color: theme.colorScheme.primary),
    );

    // Test stop indicator radius larger than minHeight.
    await tester.pumpWidget(buildIndicator(stopIndicatorRadius: 30.0, minHeight: 20.0));
    expect(
      find.byType(LinearProgressIndicator),
      // Stop indicator radius is clamped to minHeight.
      paints..circle(x: 190.0, y: 10.0, radius: 10.0, color: theme.colorScheme.primary),
    );
  });

  testWidgets('LinearProgressIndicator default track gap when year2023 is false', (
    WidgetTester tester,
  ) async {
    const double defaultTrackGap = 4.0;
    Widget buildIndicator({required TextDirection textDirection}) {
      return Directionality(
        textDirection: textDirection,
        child: const Center(
          child: SizedBox(
            width: 200.0,
            child: LinearProgressIndicator(year2023: false, value: 0.5),
          ),
        ),
      );
    }

    // Test default track gap in LTR.
    await tester.pumpWidget(buildIndicator(textDirection: TextDirection.ltr));
    expect(
      find.byType(LinearProgressIndicator),
      paints
        // Track.
        ..rrect(
          rrect: RRect.fromLTRBR(
            100.0 + defaultTrackGap,
            0.0,
            200.0,
            4.0,
            const Radius.circular(2.0),
          ),
          color: theme.colorScheme.secondaryContainer,
        )
        // Active track.
        ..rrect(
          rrect: RRect.fromLTRBR(0.0, 0.0, 100.0, 4.0, const Radius.circular(2.0)),
          color: theme.colorScheme.primary,
        ),
    );

    // Test default track gap in RTL.
    await tester.pumpWidget(buildIndicator(textDirection: TextDirection.rtl));
    expect(
      find.byType(LinearProgressIndicator),
      paints
        // Track.
        ..rrect(
          rrect: RRect.fromLTRBR(
            0.0,
            0.0,
            100.0 - defaultTrackGap,
            4.0,
            const Radius.circular(2.0),
          ),
          color: theme.colorScheme.secondaryContainer,
        )
        // Active track.
        ..rrect(
          rrect: RRect.fromLTRBR(100.0, 0.0, 200.0, 4.0, const Radius.circular(2.0)),
          color: theme.colorScheme.primary,
        ),
    );
  });

  testWidgets('Can customise LinearProgressIndicator track gap when year2023 is false', (
    WidgetTester tester,
  ) async {
    const double customTrackGap = 12.0;
    const double noTrackGap = 0.0;
    Widget buildIndicator({double? trackGap}) {
      return Directionality(
        textDirection: TextDirection.ltr,
        child: Center(
          child: SizedBox(
            width: 200.0,
            child: LinearProgressIndicator(year2023: false, trackGap: trackGap, value: 0.5),
          ),
        ),
      );
    }

    // Test customized track gap.
    await tester.pumpWidget(buildIndicator(trackGap: customTrackGap));
    expect(
      find.byType(LinearProgressIndicator),
      paints
        // Track.
        ..rrect(
          rrect: RRect.fromLTRBR(
            100.0 + customTrackGap,
            0.0,
            200.0,
            4.0,
            const Radius.circular(2.0),
          ),
          color: theme.colorScheme.secondaryContainer,
        )
        // Stop indicator.
        ..circle()
        // Active track.
        ..rrect(
          rrect: RRect.fromLTRBR(0.0, 0.0, 100.0, 4.0, const Radius.circular(2.0)),
          color: theme.colorScheme.primary,
        ),
    );

    // Remove track gap.
    await tester.pumpWidget(buildIndicator(trackGap: noTrackGap));
    expect(
      find.byType(LinearProgressIndicator),
      paints
        // Track.
        ..rrect(
          rrect: RRect.fromLTRBR(0.0, 0.0, 200.0, 4.0, const Radius.circular(2.0)),
          color: theme.colorScheme.secondaryContainer,
        )
        // Stop indicator.
        ..circle()
        // Active indicator.
        ..rrect(
          rrect: RRect.fromLTRBR(0.0, 0.0, 100.0, 4.0, const Radius.circular(2.0)),
          color: theme.colorScheme.primary,
        ),
    );
  });

  testWidgets('Default determinate CircularProgressIndicator when year2023 is false', (
    WidgetTester tester,
  ) async {
    const EdgeInsetsGeometry padding = EdgeInsets.all(4.0);
    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: const Center(child: CircularProgressIndicator(year2023: false, value: 0.5)),
      ),
    );

    final Size indicatorBoxSize = tester.getSize(
      find.descendant(
        of: find.byType(CircularProgressIndicator),
        matching: find.byType(ConstrainedBox),
      ),
    );
    expect(
      tester.getSize(find.byType(CircularProgressIndicator)),
      equals(
        Size(
          indicatorBoxSize.width + padding.horizontal,
          indicatorBoxSize.height + padding.vertical,
        ),
      ),
    );
    expect(
      find.byType(CircularProgressIndicator),
      paints
        // Track.
        ..arc(
          rect: const Rect.fromLTRB(2.0, 2.0, 38.0, 38.0),
          color: theme.colorScheme.secondaryContainer,
          strokeWidth: 4.0,
          strokeCap: StrokeCap.round,
          style: PaintingStyle.stroke,
        )
        // Active indicator.
        ..arc(
          rect: const Rect.fromLTRB(2.0, 2.0, 38.0, 38.0),
          color: theme.colorScheme.primary,
          strokeWidth: 4.0,
          strokeCap: StrokeCap.round,
          style: PaintingStyle.stroke,
        ),
    );
    await expectLater(
      find.byType(CircularProgressIndicator),
      matchesGoldenFile('circular_progress_indicator_determinate_year2023_false.png'),
    );
  });

  testWidgets('Default indeterminate CircularProgressIndicator when year2023 is false', (
    WidgetTester tester,
  ) async {
    const EdgeInsetsGeometry padding = EdgeInsets.all(4.0);
    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: const Center(child: CircularProgressIndicator(year2023: false)),
      ),
    );

    // Advance the animation.
    await tester.pump(const Duration(milliseconds: 200));

    final Size indicatorBoxSize = tester.getSize(
      find.descendant(
        of: find.byType(CircularProgressIndicator),
        matching: find.byType(ConstrainedBox),
      ),
    );
    expect(
      tester.getSize(find.byType(CircularProgressIndicator)),
      equals(
        Size(
          indicatorBoxSize.width + padding.horizontal,
          indicatorBoxSize.height + padding.vertical,
        ),
      ),
    );
    expect(
      find.byType(CircularProgressIndicator),
      paints
        // Active indicator.
        ..arc(
          rect: const Rect.fromLTRB(2.0, 2.0, 38.0, 38.0),
          color: theme.colorScheme.primary,
          strokeWidth: 4.0,
          strokeCap: StrokeCap.round,
          style: PaintingStyle.stroke,
        ),
    );
    await expectLater(
      find.byType(CircularProgressIndicator),
      matchesGoldenFile('circular_progress_indicator_indeterminate_year2023_false.png'),
    );
  });

  testWidgets('CircularProgressIndicator track gap can be adjusted when year2023 is false', (
    WidgetTester tester,
  ) async {
    Widget buildIndicator({double? trackGap}) {
      return MaterialApp(
        home: Center(
          child: CircularProgressIndicator(year2023: false, trackGap: trackGap, value: 0.5),
        ),
      );
    }

    await tester.pumpWidget(buildIndicator());
    await expectLater(
      find.byType(CircularProgressIndicator),
      matchesGoldenFile('circular_progress_indicator_default_track_gap_year2023_false.png'),
    );

    await tester.pumpWidget(buildIndicator(trackGap: 12.0));
    await expectLater(
      find.byType(CircularProgressIndicator),
      matchesGoldenFile('circular_progress_indicator_custom_track_gap_year2023_false.png'),
    );

    await tester.pumpWidget(buildIndicator(trackGap: 0.0));
    await expectLater(
      find.byType(CircularProgressIndicator),
      matchesGoldenFile('circular_progress_indicator_no_track_gap_year2023_false.png'),
    );
  });

  testWidgets('Can override CircularProgressIndicator stroke cap when year2023 is false', (
    WidgetTester tester,
  ) async {
    const StrokeCap strokeCap = StrokeCap.square;
    await tester.pumpWidget(
      const MaterialApp(
        home: Center(
          child: CircularProgressIndicator(year2023: false, strokeCap: strokeCap, value: 0.5),
        ),
      ),
    );

    expect(
      find.byType(CircularProgressIndicator),
      paints
        // Track.
        ..arc(strokeCap: strokeCap)
        // Active indicator.
        ..arc(strokeCap: strokeCap),
    );
    await expectLater(
      find.byType(CircularProgressIndicator),
      matchesGoldenFile('circular_progress_indicator_custom_stroke_cap_year2023_false.png'),
    );
  });

  testWidgets('CircularProgressIndicator.constraints can override default size', (
    WidgetTester tester,
  ) async {
    const Size size = Size(64, 64);
    await tester.pumpWidget(
      MaterialApp(
        home: Center(
          child: CircularProgressIndicator(
            constraints: BoxConstraints(minWidth: size.width, minHeight: size.height),
            value: 0.5,
          ),
        ),
      ),
    );

    expect(tester.getSize(find.byType(CircularProgressIndicator)), equals(size));
  });

  testWidgets('CircularProgressIndicator padding can be customized', (WidgetTester tester) async {
    const EdgeInsetsGeometry padding = EdgeInsets.all(12.0);
    await tester.pumpWidget(
      const MaterialApp(
        home: Center(
          child: CircularProgressIndicator(padding: padding, year2023: false, value: 0.5),
        ),
      ),
    );

    final Size indicatorBoxSize = tester.getSize(
      find.descendant(
        of: find.byType(CircularProgressIndicator),
        matching: find.byType(ConstrainedBox),
      ),
    );
    expect(
      tester.getSize(find.byType(CircularProgressIndicator)),
      equals(
        Size(
          indicatorBoxSize.width + padding.horizontal,
          indicatorBoxSize.height + padding.vertical,
        ),
      ),
    );
  });
}

class _RefreshProgressIndicatorGolden extends StatefulWidget {
  const _RefreshProgressIndicatorGolden();

  @override
  _RefreshProgressIndicatorGoldenState createState() => _RefreshProgressIndicatorGoldenState();
}

class _RefreshProgressIndicatorGoldenState extends State<_RefreshProgressIndicatorGolden>
    with SingleTickerProviderStateMixin {
  late final AnimationController controller =
      AnimationController(vsync: this, duration: const Duration(seconds: 1))
        ..forward()
        ..addListener(() {
          setState(() {});
        })
        ..addStatusListener((AnimationStatus status) {
          if (status.isCompleted) {
            indeterminate = true;
          }
        });

  bool indeterminate = false;

  @override
  void dispose() {
    controller.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Center(
      child: Directionality(
        textDirection: TextDirection.ltr,
        child: RefreshProgressIndicator(value: indeterminate ? null : controller.value),
      ),
    );
  }
}
