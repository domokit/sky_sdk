// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This file is run as part of a reduced test set in CI on Mac and Windows
// machines.
@Tags(<String>['reduced-test-set'])
library;

import 'package:flutter/cupertino.dart';
import 'package:flutter/foundation.dart';
import 'package:flutter_test/flutter_test.dart';

class _FilterTest extends StatelessWidget {
  const _FilterTest(Widget child, {this.brightness = Brightness.light}) : _child = child;
  final Brightness brightness;
  final Widget _child;

  @override
  Widget build(BuildContext context) {
    final Size size = MediaQuery.sizeOf(context);
    final double tileHeight = size.height / 4;
    final double tileWidth = size.width / 8;
    return CupertinoApp(
      home: Stack(
        fit: StackFit.expand,
        children: <Widget>[
          // 512 color tiles
          // 4 alpha levels (0.416, 0.25, 0.5, 0.75)
          for (int a = 0; a < 4; a++)
            for (int h = 0; h < 8; h++) // 8 hues
              for (int s = 0; s < 4; s++) // 4 saturation levels
                for (int b = 0; b < 4; b++) // 4 brightness levels
                  Positioned(
                    left: h * tileWidth + b * tileWidth / 4,
                    top: a * tileHeight + s * tileHeight / 4,
                    height: tileHeight,
                    width: tileWidth,
                    child: ColoredBox(
                      color:
                          HSVColor.fromAHSV(
                            0.5 + a / 8,
                            h * 45,
                            0.5 + s / 8,
                            0.5 + b / 8,
                          ).toColor(),
                    ),
                  ),
          Padding(
            padding: const EdgeInsets.all(32),
            child: CupertinoTheme(data: CupertinoThemeData(brightness: brightness), child: _child),
          ),
        ],
      ),
    );
  }
}

void main() {
  void disableVibranceForTest() {
    CupertinoPopupSurface.debugIsVibrancePainted = false;
    addTearDown(() {
      CupertinoPopupSurface.debugIsVibrancePainted = true;
    });
  }

  // Golden displays the color filter effect of the CupertinoPopupSurface
  // when the ambient brightness is light.
  testWidgets(
    'Brightness.light color filter',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        const _FilterTest(
          CupertinoPopupSurface(blurSigma: 0, isSurfacePainted: false, child: SizedBox()),
        ),
      );

      await expectLater(
        find.byType(CupertinoApp),
        matchesGoldenFile('cupertinoPopupSurface.color-filter.light.png'),
      );
    },
    skip: kIsWasm, // https://github.com/flutter/flutter/issues/152026
  );

  // Golden displays the color filter effect of the CupertinoPopupSurface
  // when the ambient brightness is dark.
  testWidgets(
    'Brightness.dark color filter',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        const _FilterTest(
          CupertinoPopupSurface(blurSigma: 0, isSurfacePainted: false, child: SizedBox()),
          brightness: Brightness.dark,
        ),
      );

      await expectLater(
        find.byType(CupertinoApp),
        matchesGoldenFile('cupertinoPopupSurface.color-filter.dark.png'),
      );
    },
    skip: kIsWasm, // https://github.com/flutter/flutter/issues/152026
  );

  // Golden displays color tiles without CupertinoPopupSurface being
  // displayed.
  testWidgets('Setting debugIsVibrancePainted to false removes the color filter', (
    WidgetTester tester,
  ) async {
    disableVibranceForTest();
    await tester.pumpWidget(
      const _FilterTest(
        CupertinoPopupSurface(blurSigma: 0, isSurfacePainted: false, child: SizedBox()),
      ),
    );

    // The BackdropFilter widget should not be mounted when blurSigma is 0 and
    // CupertinoPopupSurface.debugIsVibrancePainted is false.
    expect(
      find.descendant(
        of: find.byType(CupertinoPopupSurface),
        matching: find.byType(BackdropFilter),
      ),
      findsNothing,
    );

    await expectLater(
      find.byType(CupertinoApp),
      matchesGoldenFile('cupertinoPopupSurface.color-filter.removed.png'),
    );
  });

  // Golden displays the surface color of the CupertinoPopupSurface
  // in light mode.
  testWidgets('Brightness.light surface color', (WidgetTester tester) async {
    disableVibranceForTest();
    await tester.pumpWidget(
      const _FilterTest(CupertinoPopupSurface(blurSigma: 0, child: SizedBox())),
    );

    await expectLater(
      find.byType(CupertinoApp),
      matchesGoldenFile('cupertinoPopupSurface.surface-color.light.png'),
    );
  });

  // Golden displays the surface color of the CupertinoPopupSurface
  // in dark mode.
  testWidgets('Brightness.dark surface color', (WidgetTester tester) async {
    disableVibranceForTest();
    await tester.pumpWidget(
      const _FilterTest(
        CupertinoPopupSurface(blurSigma: 0, child: SizedBox()),
        brightness: Brightness.dark,
      ),
    );

    await expectLater(
      find.byType(CupertinoApp),
      matchesGoldenFile('cupertinoPopupSurface.surface-color.dark.png'),
    );
  });

  // Golden displays a CupertinoPopupSurface with the color removed. The result
  // should only display color tiles.
  testWidgets('Setting isSurfacePainted to false removes the surface color', (
    WidgetTester tester,
  ) async {
    disableVibranceForTest();
    await tester.pumpWidget(
      const _FilterTest(
        CupertinoPopupSurface(blurSigma: 0, isSurfacePainted: false, child: SizedBox()),
        brightness: Brightness.dark,
      ),
    );

    await expectLater(
      find.byType(CupertinoApp),
      matchesGoldenFile('cupertinoPopupSurface.surface-color.removed.png'),
    );
  });

  // Goldens display a CupertinoPopupSurface with no vibrance or surface
  // color, with blur sigmas of 5 and 30 (default).
  testWidgets(
    'Positive blurSigma applies blur',
    (WidgetTester tester) async {
      disableVibranceForTest();
      await tester.pumpWidget(
        const _FilterTest(
          CupertinoPopupSurface(isSurfacePainted: false, blurSigma: 5, child: SizedBox()),
        ),
      );

      await expectLater(
        find.byType(CupertinoApp),
        matchesGoldenFile('cupertinoPopupSurface.blur.5.png'),
      );

      await tester.pumpWidget(
        const _FilterTest(CupertinoPopupSurface(isSurfacePainted: false, child: SizedBox())),
      );

      await expectLater(
        find.byType(CupertinoApp),
        // 30 is the default blur sigma
        matchesGoldenFile('cupertinoPopupSurface.blur.30.png'),
      );
    },
    skip: kIsWasm, // https://github.com/flutter/flutter/issues/152026
  );

  // Golden displays a CupertinoPopupSurface with a blur sigma of 0. Because
  // the blur sigma is 0 and vibrance and surface are not painted, no popup
  // surface is displayed.
  testWidgets('Setting blurSigma to zero removes blur', (WidgetTester tester) async {
    disableVibranceForTest();
    await tester.pumpWidget(
      const _FilterTest(
        CupertinoPopupSurface(isSurfacePainted: false, blurSigma: 0, child: SizedBox()),
      ),
    );

    // The BackdropFilter widget should not be mounted when blurSigma is 0 and
    // CupertinoPopupSurface.isVibrancePainted is false.
    expect(
      find.descendant(
        of: find.byType(CupertinoPopupSurface),
        matching: find.byType(BackdropFilter),
      ),
      findsNothing,
    );

    await expectLater(
      find.byType(CupertinoApp),
      matchesGoldenFile('cupertinoPopupSurface.blur.0.png'),
    );

    await tester.pumpWidget(
      const _FilterTest(
        CupertinoPopupSurface(isSurfacePainted: false, blurSigma: 0, child: SizedBox()),
      ),
    );
  });

  testWidgets('Setting a blurSigma to a negative number throws', (WidgetTester tester) async {
    try {
      disableVibranceForTest();
      await tester.pumpWidget(
        _FilterTest(
          CupertinoPopupSurface(isSurfacePainted: false, blurSigma: -1, child: const SizedBox()),
        ),
      );

      fail('CupertinoPopupSurface did not throw when provided a negative blur sigma.');
    } on AssertionError catch (error) {
      expect(
        error.toString(),
        contains('CupertinoPopupSurface requires a non-negative blur sigma.'),
      );
    }
  });

  // Regression test for https://github.com/flutter/flutter/issues/154887.
  testWidgets(
    "Applying a FadeTransition to the CupertinoPopupSurface doesn't cause transparency",
    (WidgetTester tester) async {
      final AnimationController controller = AnimationController(
        duration: const Duration(milliseconds: 100),
        vsync: const TestVSync(),
      );
      addTearDown(controller.dispose);
      controller.forward();

      await tester.pumpWidget(
        _FilterTest(
          FadeTransition(
            opacity: controller,
            child: const CupertinoPopupSurface(child: SizedBox()),
          ),
        ),
      );

      await tester.pump(const Duration(milliseconds: 50));

      // Golden should display a CupertinoPopupSurface with no transparency
      // directly underneath the surface. A small amount of transparency should be
      // present on the upper-left corner of the screen.
      //
      // If transparency (gray and white grid) is present underneath the surface,
      // the blendmode is being incorrectly applied.
      await expectLater(
        find.byType(CupertinoApp),
        matchesGoldenFile('cupertinoPopupSurface.blendmode-fix.0.png'),
      );

      await tester.pumpAndSettle();
    },
    variant: TargetPlatformVariant.only(TargetPlatform.iOS),
  );

  // Golden displays a CupertinoPopupSurface with all enabled features.
  //
  // CupertinoPopupSurface uses ImageFilter.compose, which applies an inner
  // filter first, followed by an outer filter (e.g. result =
  // outer(inner(source))).
  //
  // For CupertinoPopupSurface, this means that the pixels underlying the
  // surface are first saturated with a ColorFilter, and the resulting saturated
  // pixels are blurred with an ImageFilter.blur. This test verifies that this
  // order does not change.
  testWidgets('Saturation is applied before blur', (WidgetTester tester) async {
    await tester.pumpWidget(const _FilterTest(CupertinoPopupSurface(child: SizedBox())));

    await expectLater(
      find.byType(CupertinoApp),
      matchesGoldenFile('cupertinoPopupSurface.composition.png'),
    );

    disableVibranceForTest();
    await tester.pumpWidget(
      const _FilterTest(
        Stack(
          fit: StackFit.expand,
          children: <Widget>[
            CupertinoPopupSurface(isSurfacePainted: false, blurSigma: 0, child: SizedBox()),
            CupertinoPopupSurface(child: SizedBox()),
          ],
        ),
      ),
    );

    await expectLater(
      find.byType(CupertinoApp),
      matchesGoldenFile('cupertinoPopupSurface.composition.png'),
    );
  });
}
