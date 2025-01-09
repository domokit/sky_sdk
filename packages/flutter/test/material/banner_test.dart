// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('MaterialBanner properties are respected', (WidgetTester tester) async {
    const String contentText = 'Content';
    const Color backgroundColor = Colors.pink;
    const Color surfaceTintColor = Colors.green;
    const Color shadowColor = Colors.blue;
    const Color dividerColor = Colors.yellow;
    const TextStyle contentTextStyle = TextStyle(color: Colors.pink);

    await tester.pumpWidget(
      MaterialApp(
        home: MaterialBanner(
          backgroundColor: backgroundColor,
          surfaceTintColor: surfaceTintColor,
          shadowColor: shadowColor,
          dividerColor: dividerColor,
          contentTextStyle: contentTextStyle,
          content: const Text(contentText),
          actions: <Widget>[TextButton(child: const Text('Action'), onPressed: () {})],
        ),
      ),
    );

    final Material material = _getMaterialFromBanner(tester);
    expect(material.elevation, 0.0);
    expect(material.color, backgroundColor);
    expect(material.surfaceTintColor, surfaceTintColor);
    expect(material.shadowColor, shadowColor);

    final RenderParagraph content = _getTextRenderObjectFromDialog(tester, contentText);
    expect(content.text.style, contentTextStyle);

    final Divider divider = tester.widget<Divider>(find.byType(Divider));
    expect(divider.color, dividerColor);
  });

  testWidgets('MaterialBanner properties are respected when presented by ScaffoldMessenger', (
    WidgetTester tester,
  ) async {
    const String contentText = 'Content';
    const Key tapTarget = Key('tap-target');
    const Color backgroundColor = Colors.pink;
    const Color surfaceTintColor = Colors.green;
    const Color shadowColor = Colors.blue;
    const Color dividerColor = Colors.yellow;
    const TextStyle contentTextStyle = TextStyle(color: Colors.pink);

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Builder(
            builder: (BuildContext context) {
              return GestureDetector(
                key: tapTarget,
                onTap: () {
                  ScaffoldMessenger.of(context).showMaterialBanner(
                    MaterialBanner(
                      content: const Text(contentText),
                      backgroundColor: backgroundColor,
                      surfaceTintColor: surfaceTintColor,
                      shadowColor: shadowColor,
                      dividerColor: dividerColor,
                      contentTextStyle: contentTextStyle,
                      actions: <Widget>[
                        TextButton(
                          child: const Text('DISMISS'),
                          onPressed:
                              () => ScaffoldMessenger.of(context).hideCurrentMaterialBanner(),
                        ),
                      ],
                    ),
                  );
                },
                behavior: HitTestBehavior.opaque,
                child: const SizedBox(height: 100.0, width: 100.0),
              );
            },
          ),
        ),
      ),
    );
    await tester.tap(find.byKey(tapTarget));
    await tester.pumpAndSettle();

    final Material material = _getMaterialFromText(tester, contentText);
    expect(material.elevation, 0.0);
    expect(material.color, backgroundColor);
    expect(material.surfaceTintColor, surfaceTintColor);
    expect(material.shadowColor, shadowColor);

    final RenderParagraph content = _getTextRenderObjectFromDialog(tester, contentText);
    expect(content.text.style, contentTextStyle);

    final Divider divider = tester.widget<Divider>(find.byType(Divider));
    expect(divider.color, dividerColor);
  });

  testWidgets('Actions laid out below content if more than one action', (
    WidgetTester tester,
  ) async {
    const String contentText = 'Content';

    await tester.pumpWidget(
      MaterialApp(
        home: MaterialBanner(
          content: const Text(contentText),
          actions: <Widget>[
            TextButton(child: const Text('Action 1'), onPressed: () {}),
            TextButton(child: const Text('Action 2'), onPressed: () {}),
          ],
        ),
      ),
    );

    final Offset contentBottomLeft = tester.getBottomLeft(find.text(contentText));
    final Offset actionsTopLeft = tester.getTopLeft(find.byType(OverflowBar));
    expect(contentBottomLeft.dy, lessThan(actionsTopLeft.dy));
    expect(contentBottomLeft.dx, lessThan(actionsTopLeft.dx));
  });

  testWidgets(
    'Actions laid out below content if more than one action when presented by ScaffoldMessenger',
    (WidgetTester tester) async {
      const String contentText = 'Content';
      const Key tapTarget = Key('tap-target');
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: Builder(
              builder: (BuildContext context) {
                return GestureDetector(
                  key: tapTarget,
                  onTap: () {
                    ScaffoldMessenger.of(context).showMaterialBanner(
                      MaterialBanner(
                        content: const Text(contentText),
                        actions: <Widget>[
                          TextButton(
                            child: const Text('OK'),
                            onPressed:
                                () => ScaffoldMessenger.of(context).hideCurrentMaterialBanner(),
                          ),
                          TextButton(
                            child: const Text('DISMISS'),
                            onPressed:
                                () => ScaffoldMessenger.of(context).hideCurrentMaterialBanner(),
                          ),
                        ],
                      ),
                    );
                  },
                  behavior: HitTestBehavior.opaque,
                  child: const SizedBox(height: 100.0, width: 100.0),
                );
              },
            ),
          ),
        ),
      );
      await tester.tap(find.byKey(tapTarget));
      await tester.pumpAndSettle();

      final Offset contentBottomLeft = tester.getBottomLeft(find.text(contentText));
      final Offset actionsTopLeft = tester.getTopLeft(find.byType(OverflowBar));
      expect(contentBottomLeft.dy, lessThan(actionsTopLeft.dy));
      expect(contentBottomLeft.dx, lessThan(actionsTopLeft.dx));
    },
  );

  testWidgets('Actions laid out beside content if only one action', (WidgetTester tester) async {
    const String contentText = 'Content';

    await tester.pumpWidget(
      MaterialApp(
        home: MaterialBanner(
          content: const Text(contentText),
          actions: <Widget>[TextButton(child: const Text('Action'), onPressed: () {})],
        ),
      ),
    );

    final Offset contentBottomLeft = tester.getBottomLeft(find.text(contentText));
    final Offset actionsTopRight = tester.getTopRight(find.byType(OverflowBar));
    expect(contentBottomLeft.dy, greaterThan(actionsTopRight.dy));
    expect(contentBottomLeft.dx, lessThan(actionsTopRight.dx));
  });

  testWidgets(
    'Actions laid out beside content if only one action when presented by ScaffoldMessenger',
    (WidgetTester tester) async {
      const String contentText = 'Content';
      const Key tapTarget = Key('tap-target');
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: Builder(
              builder: (BuildContext context) {
                return GestureDetector(
                  key: tapTarget,
                  onTap: () {
                    ScaffoldMessenger.of(context).showMaterialBanner(
                      MaterialBanner(
                        content: const Text(contentText),
                        actions: <Widget>[
                          TextButton(
                            child: const Text('DISMISS'),
                            onPressed:
                                () => ScaffoldMessenger.of(context).hideCurrentMaterialBanner(),
                          ),
                        ],
                      ),
                    );
                  },
                  behavior: HitTestBehavior.opaque,
                  child: const SizedBox(height: 100.0, width: 100.0),
                );
              },
            ),
          ),
        ),
      );
      await tester.tap(find.byKey(tapTarget));
      await tester.pumpAndSettle();

      final Offset contentBottomLeft = tester.getBottomLeft(find.text(contentText));
      final Offset actionsTopRight = tester.getTopRight(find.byType(OverflowBar));
      expect(contentBottomLeft.dy, greaterThan(actionsTopRight.dy));
      expect(contentBottomLeft.dx, lessThan(actionsTopRight.dx));
    },
  );

  testWidgets('material banner content can scale and has maxScaleFactor', (
    WidgetTester tester,
  ) async {
    const String label = 'A';
    Widget buildApp({required TextScaler textScaler}) {
      return MaterialApp(
        home: MediaQuery(
          data: MediaQueryData(textScaler: textScaler),
          child: MaterialBanner(
            forceActionsBelow: true,
            content: const SizedBox(child: Center(child: Text(label))),
            actions: <Widget>[TextButton(child: const Text('B'), onPressed: () {})],
          ),
        ),
      );
    }

    await tester.pumpWidget(buildApp(textScaler: TextScaler.noScaling));
    expect(find.text(label), findsOneWidget);

    if (!kIsWeb || isSkiaWeb) {
      // https://github.com/flutter/flutter/issues/99933
      expect(tester.getSize(find.text(label)), const Size(14.25, 20.0));
    }

    await tester.pumpWidget(buildApp(textScaler: const TextScaler.linear(1.1)));
    await tester.pumpAndSettle();
    if (!kIsWeb || isSkiaWeb) {
      // https://github.com/flutter/flutter/issues/99933
      expect(_sizeAlmostEqual(tester.getSize(find.text(label)), const Size(15.65, 22.0)), true);
    }

    await tester.pumpWidget(buildApp(textScaler: const TextScaler.linear(1.5)));
    if (!kIsWeb || isSkiaWeb) {
      // https://github.com/flutter/flutter/issues/99933
      expect(_sizeAlmostEqual(tester.getSize(find.text(label)), const Size(21.25, 30)), true);
    }

    await tester.pumpWidget(buildApp(textScaler: const TextScaler.linear(4)));
    if (!kIsWeb || isSkiaWeb) {
      // https://github.com/flutter/flutter/issues/99933
      expect(_sizeAlmostEqual(tester.getSize(find.text(label)), const Size(21.25, 30)), true);
    }
  });

  group('MaterialBanner elevation', () {
    Widget buildBanner(Key tapTarget, {double? elevation, double? themeElevation}) {
      return MaterialApp(
        theme: ThemeData(bannerTheme: MaterialBannerThemeData(elevation: themeElevation)),
        home: Scaffold(
          body: Builder(
            builder: (BuildContext context) {
              return GestureDetector(
                key: tapTarget,
                onTap: () {
                  ScaffoldMessenger.of(context).showMaterialBanner(
                    MaterialBanner(
                      content: const Text('MaterialBanner'),
                      elevation: elevation,
                      actions: <Widget>[
                        TextButton(
                          child: const Text('DISMISS'),
                          onPressed:
                              () => ScaffoldMessenger.of(context).hideCurrentMaterialBanner(),
                        ),
                      ],
                    ),
                  );
                },
                behavior: HitTestBehavior.opaque,
                child: const SizedBox(height: 100.0, width: 100.0),
              );
            },
          ),
        ),
      );
    }

    testWidgets('Elevation defaults to 0', (WidgetTester tester) async {
      const Key tapTarget = Key('tap-target');

      await tester.pumpWidget(buildBanner(tapTarget));
      await tester.tap(find.byKey(tapTarget));
      await tester.pumpAndSettle();
      expect(_getMaterialFromBanner(tester).elevation, 0.0);
      await tester.tap(find.text('DISMISS'));
      await tester.pumpAndSettle();

      await tester.pumpWidget(buildBanner(tapTarget, themeElevation: 6.0));
      await tester.tap(find.byKey(tapTarget));
      await tester.pumpAndSettle();
      expect(_getMaterialFromBanner(tester).elevation, 6.0);
      await tester.tap(find.text('DISMISS'));
      await tester.pumpAndSettle();

      await tester.pumpWidget(buildBanner(tapTarget, elevation: 3.0, themeElevation: 6.0));
      await tester.tap(find.byKey(tapTarget));
      await tester.pumpAndSettle();
      expect(_getMaterialFromBanner(tester).elevation, 3.0);
      await tester.tap(find.text('DISMISS'));
      await tester.pumpAndSettle();
    });

    testWidgets('Uses elevation of MaterialBannerTheme by default', (WidgetTester tester) async {
      const Key tapTarget = Key('tap-target');

      await tester.pumpWidget(buildBanner(tapTarget, themeElevation: 6.0));
      await tester.tap(find.byKey(tapTarget));
      await tester.pumpAndSettle();
      expect(_getMaterialFromBanner(tester).elevation, 6.0);
      await tester.tap(find.text('DISMISS'));
      await tester.pumpAndSettle();

      await tester.pumpWidget(buildBanner(tapTarget, elevation: 3.0, themeElevation: 6.0));
      await tester.tap(find.byKey(tapTarget));
      await tester.pumpAndSettle();
      expect(_getMaterialFromBanner(tester).elevation, 3.0);
      await tester.tap(find.text('DISMISS'));
      await tester.pumpAndSettle();
    });

    testWidgets('Scaffold body is pushed down if elevation is 0', (WidgetTester tester) async {
      const Key tapTarget = Key('tap-target');

      await tester.pumpWidget(buildBanner(tapTarget, elevation: 0.0));
      await tester.tap(find.byKey(tapTarget));
      await tester.pumpAndSettle();

      final Offset contentTopLeft = tester.getTopLeft(find.byKey(tapTarget));
      final Offset bannerBottomLeft = tester.getBottomLeft(find.byType(MaterialBanner));

      expect(contentTopLeft.dx, 0.0);
      expect(contentTopLeft.dy, greaterThanOrEqualTo(bannerBottomLeft.dy));
    });
  });

  testWidgets('MaterialBanner control test', (WidgetTester tester) async {
    const String helloMaterialBanner = 'Hello MaterialBanner';
    const Key tapTarget = Key('tap-target');
    const Key dismissTarget = Key('dismiss-target');
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Builder(
            builder: (BuildContext context) {
              return GestureDetector(
                key: tapTarget,
                onTap: () {
                  ScaffoldMessenger.of(context).showMaterialBanner(
                    MaterialBanner(
                      content: const Text(helloMaterialBanner),
                      actions: <Widget>[
                        TextButton(
                          key: dismissTarget,
                          child: const Text('DISMISS'),
                          onPressed:
                              () => ScaffoldMessenger.of(context).hideCurrentMaterialBanner(),
                        ),
                      ],
                    ),
                  );
                },
                behavior: HitTestBehavior.opaque,
                child: const SizedBox(height: 100.0, width: 100.0),
              );
            },
          ),
        ),
      ),
    );
    expect(find.text(helloMaterialBanner), findsNothing);
    await tester.tap(find.byKey(tapTarget));
    expect(find.text(helloMaterialBanner), findsNothing);
    await tester.pump(); // schedule animation
    expect(find.text(helloMaterialBanner), findsOneWidget);
    await tester.pump(); // begin animation
    expect(find.text(helloMaterialBanner), findsOneWidget);
    await tester.pump(
      const Duration(milliseconds: 750),
    ); // 0.75s // animation last frame; two second timer starts here
    expect(find.text(helloMaterialBanner), findsOneWidget);
    await tester.pump(const Duration(milliseconds: 750)); // 1.50s
    expect(find.text(helloMaterialBanner), findsOneWidget);
    await tester.pump(const Duration(milliseconds: 750)); // 2.25s
    expect(find.text(helloMaterialBanner), findsOneWidget);
    await tester.tap(find.byKey(dismissTarget));
    await tester.pump(); // begin animation
    expect(find.text(helloMaterialBanner), findsOneWidget); // frame 0 of dismiss animation
    await tester
        .pumpAndSettle(); // 3.75s // last frame of animation, material banner removed from build
    expect(find.text(helloMaterialBanner), findsNothing);
  });

  testWidgets('MaterialBanner twice test', (WidgetTester tester) async {
    int materialBannerCount = 0;
    const Key tapTarget = Key('tap-target');
    const Key dismissTarget = Key('dismiss-target');
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Builder(
            builder: (BuildContext context) {
              return GestureDetector(
                key: tapTarget,
                onTap: () {
                  materialBannerCount += 1;
                  ScaffoldMessenger.of(context).showMaterialBanner(
                    MaterialBanner(
                      content: Text('banner$materialBannerCount'),
                      actions: <Widget>[
                        TextButton(
                          key: dismissTarget,
                          child: const Text('DISMISS'),
                          onPressed:
                              () => ScaffoldMessenger.of(context).hideCurrentMaterialBanner(),
                        ),
                      ],
                    ),
                  );
                },
                behavior: HitTestBehavior.opaque,
                child: const SizedBox(height: 100.0, width: 100.0),
              );
            },
          ),
        ),
      ),
    );
    expect(find.text('banner1'), findsNothing);
    expect(find.text('banner2'), findsNothing);
    await tester.tap(find.byKey(tapTarget)); // queue banner1
    await tester.tap(find.byKey(tapTarget)); // queue banner2
    expect(find.text('banner1'), findsNothing);
    expect(find.text('banner2'), findsNothing);
    await tester.pump(); // schedule animation for banner1
    expect(find.text('banner1'), findsOneWidget);
    expect(find.text('banner2'), findsNothing);
    await tester.pump(); // begin animation
    expect(find.text('banner1'), findsOneWidget);
    expect(find.text('banner2'), findsNothing);
    await tester.pump(const Duration(milliseconds: 750)); // 0.75s // animation last frame
    expect(find.text('banner1'), findsOneWidget);
    expect(find.text('banner2'), findsNothing);
    await tester.pump(const Duration(milliseconds: 750)); // 1.50s
    expect(find.text('banner1'), findsOneWidget);
    expect(find.text('banner2'), findsNothing);
    await tester.pump(const Duration(milliseconds: 750)); // 2.25s
    expect(find.text('banner1'), findsOneWidget);
    expect(find.text('banner2'), findsNothing);
    await tester.tap(find.byKey(dismissTarget));
    await tester.pump(); // begin animation
    expect(find.text('banner1'), findsOneWidget);
    expect(find.text('banner2'), findsNothing);
    await tester.pump(
      const Duration(milliseconds: 750),
    ); // 3.75s // last frame of animation, material banner removed from build, new material banner put in its place
    expect(find.text('banner1'), findsNothing);
    expect(find.text('banner2'), findsOneWidget);
    await tester.pump(); // begin animation
    expect(find.text('banner1'), findsNothing);
    expect(find.text('banner2'), findsOneWidget);
    await tester.pump(const Duration(milliseconds: 750)); // 4.50s // animation last frame
    expect(find.text('banner1'), findsNothing);
    expect(find.text('banner2'), findsOneWidget);
    await tester.pump(const Duration(milliseconds: 750)); // 5.25s
    expect(find.text('banner1'), findsNothing);
    expect(find.text('banner2'), findsOneWidget);
    await tester.pump(const Duration(milliseconds: 750)); // 6.00s
    expect(find.text('banner1'), findsNothing);
    expect(find.text('banner2'), findsOneWidget);
    await tester.tap(find.byKey(dismissTarget)); // reverse animation is scheduled
    await tester.pump(); // begin animation
    expect(find.text('banner1'), findsNothing);
    expect(find.text('banner2'), findsOneWidget);
    await tester.pump(
      const Duration(milliseconds: 750),
    ); // 7.50s // last frame of animation, material banner removed from build
    expect(find.text('banner1'), findsNothing);
    expect(find.text('banner2'), findsNothing);
  });

  testWidgets('ScaffoldMessenger does not duplicate a MaterialBanner when presenting a SnackBar.', (
    WidgetTester tester,
  ) async {
    const Key materialBannerTapTarget = Key('materialbanner-tap-target');
    const Key snackBarTapTarget = Key('snackbar-tap-target');
    const String snackBarText = 'SnackBar';
    const String materialBannerText = 'MaterialBanner';
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Builder(
            builder: (BuildContext context) {
              return Column(
                children: <Widget>[
                  GestureDetector(
                    key: snackBarTapTarget,
                    onTap: () {
                      ScaffoldMessenger.of(
                        context,
                      ).showSnackBar(const SnackBar(content: Text(snackBarText)));
                    },
                    behavior: HitTestBehavior.opaque,
                    child: const SizedBox(height: 100.0, width: 100.0),
                  ),
                  GestureDetector(
                    key: materialBannerTapTarget,
                    onTap: () {
                      ScaffoldMessenger.of(context).showMaterialBanner(
                        MaterialBanner(
                          content: const Text(materialBannerText),
                          actions: <Widget>[
                            TextButton(
                              child: const Text('DISMISS'),
                              onPressed:
                                  () => ScaffoldMessenger.of(context).hideCurrentMaterialBanner(),
                            ),
                          ],
                        ),
                      );
                    },
                    behavior: HitTestBehavior.opaque,
                    child: const SizedBox(height: 100.0, width: 100.0),
                  ),
                ],
              );
            },
          ),
        ),
      ),
    );
    await tester.tap(find.byKey(snackBarTapTarget));
    await tester.tap(find.byKey(materialBannerTapTarget));
    await tester.pumpAndSettle();

    expect(find.text(snackBarText), findsOneWidget);
    expect(find.text(materialBannerText), findsOneWidget);
  });

  // Regression test for https://github.com/flutter/flutter/issues/39574
  testWidgets('Single action laid out beside content but aligned to the trailing edge', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        home: MaterialBanner(
          content: const Text('Content'),
          actions: <Widget>[TextButton(child: const Text('Action'), onPressed: () {})],
        ),
      ),
    );

    final Offset actionsTopRight = tester.getTopRight(find.byType(OverflowBar));
    final Offset bannerTopRight = tester.getTopRight(find.byType(MaterialBanner));
    expect(actionsTopRight.dx + 8, bannerTopRight.dx); // actions OverflowBar is padded by 8
  });

  // Regression test for https://github.com/flutter/flutter/issues/39574
  testWidgets(
    'Single action laid out beside content but aligned to the trailing edge when presented by ScaffoldMessenger',
    (WidgetTester tester) async {
      const Key tapTarget = Key('tap-target');
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: Builder(
              builder: (BuildContext context) {
                return GestureDetector(
                  key: tapTarget,
                  onTap: () {
                    ScaffoldMessenger.of(context).showMaterialBanner(
                      MaterialBanner(
                        content: const Text('Content'),
                        actions: <Widget>[
                          TextButton(
                            child: const Text('DISMISS'),
                            onPressed:
                                () => ScaffoldMessenger.of(context).hideCurrentMaterialBanner(),
                          ),
                        ],
                      ),
                    );
                  },
                  behavior: HitTestBehavior.opaque,
                  child: const SizedBox(height: 100.0, width: 100.0),
                );
              },
            ),
          ),
        ),
      );
      await tester.tap(find.byKey(tapTarget));
      await tester.pumpAndSettle();

      final Offset actionsTopRight = tester.getTopRight(find.byType(OverflowBar));
      final Offset bannerTopRight = tester.getTopRight(find.byType(MaterialBanner));
      expect(actionsTopRight.dx + 8, bannerTopRight.dx); // actions OverflowBar is padded by 8
    },
  );

  // Regression test for https://github.com/flutter/flutter/issues/39574
  testWidgets('Single action laid out beside content but aligned to the trailing edge - RTL', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Directionality(
          textDirection: TextDirection.rtl,
          child: MaterialBanner(
            content: const Text('Content'),
            actions: <Widget>[TextButton(child: const Text('Action'), onPressed: () {})],
          ),
        ),
      ),
    );

    final Offset actionsTopLeft = tester.getTopLeft(find.byType(OverflowBar));
    final Offset bannerTopLeft = tester.getTopLeft(find.byType(MaterialBanner));
    expect(
      actionsTopLeft.dx - 8,
      moreOrLessEquals(bannerTopLeft.dx),
    ); // actions OverflowBar is padded by 8
  });

  testWidgets(
    'Single action laid out beside content but aligned to the trailing edge when presented by ScaffoldMessenger - RTL',
    (WidgetTester tester) async {
      const Key tapTarget = Key('tap-target');
      await tester.pumpWidget(
        MaterialApp(
          home: Directionality(
            textDirection: TextDirection.rtl,
            child: Scaffold(
              body: Builder(
                builder: (BuildContext context) {
                  return GestureDetector(
                    key: tapTarget,
                    onTap: () {
                      ScaffoldMessenger.of(context).showMaterialBanner(
                        MaterialBanner(
                          content: const Text('Content'),
                          actions: <Widget>[
                            TextButton(
                              child: const Text('DISMISS'),
                              onPressed:
                                  () => ScaffoldMessenger.of(context).hideCurrentMaterialBanner(),
                            ),
                          ],
                        ),
                      );
                    },
                    behavior: HitTestBehavior.opaque,
                    child: const SizedBox(height: 100.0, width: 100.0),
                  );
                },
              ),
            ),
          ),
        ),
      );
      await tester.tap(find.byKey(tapTarget));
      await tester.pumpAndSettle();

      final Offset actionsTopLeft = tester.getTopLeft(find.byType(OverflowBar));
      final Offset bannerTopLeft = tester.getTopLeft(find.byType(MaterialBanner));
      expect(
        actionsTopLeft.dx - 8,
        moreOrLessEquals(bannerTopLeft.dx),
      ); // actions OverflowBar is padded by 8
    },
  );

  testWidgets('Actions laid out below content if forced override', (WidgetTester tester) async {
    const String contentText = 'Content';

    await tester.pumpWidget(
      MaterialApp(
        home: MaterialBanner(
          forceActionsBelow: true,
          content: const Text(contentText),
          actions: <Widget>[TextButton(child: const Text('Action'), onPressed: () {})],
        ),
      ),
    );

    final Offset contentBottomLeft = tester.getBottomLeft(find.text(contentText));
    final Offset actionsTopLeft = tester.getTopLeft(find.byType(OverflowBar));
    expect(contentBottomLeft.dy, lessThan(actionsTopLeft.dy));
    expect(contentBottomLeft.dx, lessThan(actionsTopLeft.dx));
  });

  testWidgets(
    'Actions laid out below content if forced override when presented by ScaffoldMessenger',
    (WidgetTester tester) async {
      const String contentText = 'Content';
      const Key tapTarget = Key('tap-target');
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: Builder(
              builder: (BuildContext context) {
                return GestureDetector(
                  key: tapTarget,
                  onTap: () {
                    ScaffoldMessenger.of(context).showMaterialBanner(
                      MaterialBanner(
                        content: const Text(contentText),
                        forceActionsBelow: true,
                        actions: <Widget>[
                          TextButton(
                            child: const Text('DISMISS'),
                            onPressed:
                                () => ScaffoldMessenger.of(context).hideCurrentMaterialBanner(),
                          ),
                        ],
                      ),
                    );
                  },
                  behavior: HitTestBehavior.opaque,
                  child: const SizedBox(height: 100.0, width: 100.0),
                );
              },
            ),
          ),
        ),
      );
      await tester.tap(find.byKey(tapTarget));
      await tester.pumpAndSettle();

      final Offset contentBottomLeft = tester.getBottomLeft(find.text(contentText));
      final Offset actionsTopLeft = tester.getTopLeft(find.byType(OverflowBar));
      expect(contentBottomLeft.dy, lessThan(actionsTopLeft.dy));
      expect(contentBottomLeft.dx, lessThan(actionsTopLeft.dx));
    },
  );

  testWidgets('Action widgets layout', (WidgetTester tester) async {
    // This regression test ensures that the action widgets layout matches what
    // it was, before ButtonBar was replaced by OverflowBar.
    Widget buildFrame(int actionCount, TextDirection textDirection) {
      return MaterialApp(
        home: Directionality(
          textDirection: textDirection,
          child: MaterialBanner(
            content: const SizedBox(width: 100, height: 100),
            actions: List<Widget>.generate(actionCount, (int index) {
              return SizedBox(width: 64, height: 48, key: ValueKey<int>(index));
            }),
          ),
        ),
      );
    }

    final Finder action0 = find.byKey(const ValueKey<int>(0));
    final Finder action1 = find.byKey(const ValueKey<int>(1));
    final Finder action2 = find.byKey(const ValueKey<int>(2));
    // The action coordinates that follow were obtained by running
    // the test code, before ButtonBar was replaced by OverflowBar.

    await tester.pumpWidget(buildFrame(1, TextDirection.ltr));
    expect(tester.getTopLeft(action0), const Offset(728, 28));

    await tester.pumpWidget(buildFrame(1, TextDirection.rtl));
    expect(tester.getTopLeft(action0), const Offset(8, 28));

    await tester.pumpWidget(buildFrame(3, TextDirection.ltr));
    expect(tester.getTopLeft(action0), const Offset(584, 130));
    expect(tester.getTopLeft(action1), const Offset(656, 130));
    expect(tester.getTopLeft(action2), const Offset(728, 130));

    await tester.pumpWidget(buildFrame(3, TextDirection.rtl));
    expect(tester.getTopLeft(action0), const Offset(152, 130));
    expect(tester.getTopLeft(action1), const Offset(80, 130));
    expect(tester.getTopLeft(action2), const Offset(8, 130));
  });

  testWidgets('Action widgets layout when presented by ScaffoldMessenger', (
    WidgetTester tester,
  ) async {
    // This regression test ensures that the action widgets layout matches what
    // it was, before ButtonBar was replaced by OverflowBar.

    Widget buildFrame(int actionCount, TextDirection textDirection) {
      return MaterialApp(
        home: Directionality(
          textDirection: textDirection,
          child: Scaffold(
            body: Builder(
              builder: (BuildContext context) {
                return GestureDetector(
                  key: const ValueKey<String>('tap-target'),
                  onTap: () {
                    ScaffoldMessenger.of(context).showMaterialBanner(
                      MaterialBanner(
                        content: const SizedBox(width: 100, height: 100),
                        actions: List<Widget>.generate(actionCount, (int index) {
                          if (index == 0) {
                            return SizedBox(
                              width: 64,
                              height: 48,
                              key: ValueKey<int>(index),
                              child: GestureDetector(
                                key: const ValueKey<String>('dismiss-target'),
                                onTap:
                                    () => ScaffoldMessenger.of(context).hideCurrentMaterialBanner(),
                              ),
                            );
                          }

                          return SizedBox(width: 64, height: 48, key: ValueKey<int>(index));
                        }),
                      ),
                    );
                  },
                );
              },
            ),
          ),
        ),
      );
    }

    final Finder tapTarget = find.byKey(const ValueKey<String>('tap-target'));
    final Finder dismissTarget = find.byKey(const ValueKey<String>('dismiss-target'));
    final Finder action0 = find.byKey(const ValueKey<int>(0));
    final Finder action1 = find.byKey(const ValueKey<int>(1));
    final Finder action2 = find.byKey(const ValueKey<int>(2));

    // The action coordinates that follow were obtained by running
    // the test code, before ButtonBar was replaced by OverflowBar.

    await tester.pumpWidget(buildFrame(1, TextDirection.ltr));
    await tester.tap(tapTarget);
    await tester.pumpAndSettle();
    expect(tester.getTopLeft(action0), const Offset(728, 28));
    await tester.tap(dismissTarget);
    await tester.pumpAndSettle();

    await tester.pumpWidget(buildFrame(1, TextDirection.rtl));
    await tester.tap(tapTarget);
    await tester.pumpAndSettle();
    expect(tester.getTopLeft(action0), const Offset(8, 28));
    await tester.tap(dismissTarget);
    await tester.pumpAndSettle();

    await tester.pumpWidget(buildFrame(3, TextDirection.ltr));
    await tester.tap(tapTarget);
    await tester.pumpAndSettle();
    expect(tester.getTopLeft(action0), const Offset(584, 130));
    expect(tester.getTopLeft(action1), const Offset(656, 130));
    expect(tester.getTopLeft(action2), const Offset(728, 130));
    await tester.tap(dismissTarget);
    await tester.pumpAndSettle();

    await tester.pumpWidget(buildFrame(3, TextDirection.rtl));
    await tester.tap(tapTarget);
    await tester.pumpAndSettle();
    expect(tester.getTopLeft(action0), const Offset(152, 130));
    expect(tester.getTopLeft(action1), const Offset(80, 130));
    expect(tester.getTopLeft(action2), const Offset(8, 130));
    await tester.tap(dismissTarget);
    await tester.pumpAndSettle();
  });

  testWidgets('Action widgets layout with overflow', (WidgetTester tester) async {
    // This regression test ensures that the action widgets layout matches what
    // it was, before ButtonBar was replaced by OverflowBar.
    const int actionCount = 4;
    Widget buildFrame(TextDirection textDirection) {
      return MaterialApp(
        home: Directionality(
          textDirection: textDirection,
          child: MaterialBanner(
            content: const SizedBox(width: 100, height: 100),
            actions: List<Widget>.generate(actionCount, (int index) {
              return SizedBox(width: 200, height: 10, key: ValueKey<int>(index));
            }),
          ),
        ),
      );
    }
    // The action coordinates that follow were obtained by running
    // the test code, before ButtonBar was replaced by OverflowBar.

    await tester.pumpWidget(buildFrame(TextDirection.ltr));
    for (int index = 0; index < actionCount; index += 1) {
      expect(tester.getTopLeft(find.byKey(ValueKey<int>(index))), Offset(592, 134.0 + index * 10));
    }

    await tester.pumpWidget(buildFrame(TextDirection.rtl));
    for (int index = 0; index < actionCount; index += 1) {
      expect(tester.getTopLeft(find.byKey(ValueKey<int>(index))), Offset(8, 134.0 + index * 10));
    }
  });

  testWidgets('Action widgets layout with overflow when presented by ScaffoldMessenger', (
    WidgetTester tester,
  ) async {
    // This regression test ensures that the action widgets layout matches what
    // it was, before ButtonBar was replaced by OverflowBar.

    const int actionCount = 4;
    Widget buildFrame(TextDirection textDirection) {
      return MaterialApp(
        home: Directionality(
          textDirection: textDirection,
          child: Scaffold(
            body: Builder(
              builder: (BuildContext context) {
                return GestureDetector(
                  key: const ValueKey<String>('tap-target'),
                  onTap: () {
                    ScaffoldMessenger.of(context).showMaterialBanner(
                      MaterialBanner(
                        content: const SizedBox(width: 100, height: 100),
                        actions: List<Widget>.generate(actionCount, (int index) {
                          if (index == 0) {
                            return SizedBox(
                              width: 200,
                              height: 10,
                              key: ValueKey<int>(index),
                              child: GestureDetector(
                                key: const ValueKey<String>('dismiss-target'),
                                onTap:
                                    () => ScaffoldMessenger.of(context).hideCurrentMaterialBanner(),
                              ),
                            );
                          }

                          return SizedBox(width: 200, height: 10, key: ValueKey<int>(index));
                        }),
                      ),
                    );
                  },
                );
              },
            ),
          ),
        ),
      );
    }

    // The action coordinates that follow were obtained by running
    // the test code, before ButtonBar was replaced by OverflowBar.

    final Finder tapTarget = find.byKey(const ValueKey<String>('tap-target'));
    final Finder dismissTarget = find.byKey(const ValueKey<String>('dismiss-target'));

    await tester.pumpWidget(buildFrame(TextDirection.ltr));
    await tester.tap(tapTarget);
    await tester.pumpAndSettle();
    for (int index = 0; index < actionCount; index += 1) {
      expect(tester.getTopLeft(find.byKey(ValueKey<int>(index))), Offset(592, 134.0 + index * 10));
    }
    await tester.tap(dismissTarget);
    await tester.pumpAndSettle();

    await tester.pumpWidget(buildFrame(TextDirection.rtl));
    await tester.tap(tapTarget);
    await tester.pumpAndSettle();
    for (int index = 0; index < actionCount; index += 1) {
      expect(tester.getTopLeft(find.byKey(ValueKey<int>(index))), Offset(8, 134.0 + index * 10));
    }
    await tester.tap(dismissTarget);
    await tester.pumpAndSettle();
  });

  testWidgets('[overflowAlignment] test', (WidgetTester tester) async {
    const int actionCount = 4;
    Widget buildFrame(TextDirection textDirection, OverflowBarAlignment overflowAlignment) {
      return MaterialApp(
        home: Directionality(
          textDirection: textDirection,
          child: MaterialBanner(
            overflowAlignment: overflowAlignment,
            content: const SizedBox(width: 100, height: 100),
            actions: List<Widget>.generate(actionCount, (int index) {
              return SizedBox(width: 200, height: 10, key: ValueKey<int>(index));
            }),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildFrame(TextDirection.ltr, OverflowBarAlignment.start));
    for (int index = 0; index < actionCount; index += 1) {
      expect(tester.getTopLeft(find.byKey(ValueKey<int>(index))), Offset(8, 134.0 + index * 10));
    }

    await tester.pumpWidget(buildFrame(TextDirection.ltr, OverflowBarAlignment.center));
    for (int index = 0; index < actionCount; index += 1) {
      expect(tester.getTopLeft(find.byKey(ValueKey<int>(index))), Offset(300, 134.0 + index * 10));
    }

    await tester.pumpWidget(buildFrame(TextDirection.ltr, OverflowBarAlignment.end));
    for (int index = 0; index < actionCount; index += 1) {
      expect(tester.getTopLeft(find.byKey(ValueKey<int>(index))), Offset(592, 134.0 + index * 10));
    }
  });

  testWidgets('[overflowAlignment] test when presented by ScaffoldMessenger', (
    WidgetTester tester,
  ) async {
    const int actionCount = 4;
    Widget buildFrame(TextDirection textDirection, OverflowBarAlignment overflowAlignment) {
      return MaterialApp(
        home: Directionality(
          textDirection: textDirection,
          child: Scaffold(
            body: Builder(
              builder: (BuildContext context) {
                return GestureDetector(
                  key: const ValueKey<String>('tap-target'),
                  onTap: () {
                    ScaffoldMessenger.of(context).showMaterialBanner(
                      MaterialBanner(
                        overflowAlignment: overflowAlignment,
                        content: const SizedBox(width: 100, height: 100),
                        actions: List<Widget>.generate(actionCount, (int index) {
                          if (index == 0) {
                            return SizedBox(
                              width: 200,
                              height: 10,
                              key: ValueKey<int>(index),
                              child: GestureDetector(
                                key: const ValueKey<String>('dismiss-target'),
                                onTap:
                                    () => ScaffoldMessenger.of(context).hideCurrentMaterialBanner(),
                              ),
                            );
                          }

                          return SizedBox(width: 200, height: 10, key: ValueKey<int>(index));
                        }),
                      ),
                    );
                  },
                );
              },
            ),
          ),
        ),
      );
    }

    final Finder tapTarget = find.byKey(const ValueKey<String>('tap-target'));
    final Finder dismissTarget = find.byKey(const ValueKey<String>('dismiss-target'));

    await tester.pumpWidget(buildFrame(TextDirection.ltr, OverflowBarAlignment.start));
    await tester.tap(tapTarget);
    await tester.pumpAndSettle();
    for (int index = 0; index < actionCount; index += 1) {
      expect(tester.getTopLeft(find.byKey(ValueKey<int>(index))), Offset(8, 134.0 + index * 10));
    }
    await tester.tap(dismissTarget);
    await tester.pumpAndSettle();

    await tester.pumpWidget(buildFrame(TextDirection.ltr, OverflowBarAlignment.center));
    await tester.tap(tapTarget);
    await tester.pumpAndSettle();
    for (int index = 0; index < actionCount; index += 1) {
      expect(tester.getTopLeft(find.byKey(ValueKey<int>(index))), Offset(300, 134.0 + index * 10));
    }
    await tester.tap(dismissTarget);
    await tester.pumpAndSettle();

    await tester.pumpWidget(buildFrame(TextDirection.ltr, OverflowBarAlignment.end));
    await tester.tap(tapTarget);
    await tester.pumpAndSettle();
    for (int index = 0; index < actionCount; index += 1) {
      expect(tester.getTopLeft(find.byKey(ValueKey<int>(index))), Offset(592, 134.0 + index * 10));
    }
    await tester.tap(dismissTarget);
    await tester.pumpAndSettle();
  });

  testWidgets('ScaffoldMessenger will alert for MaterialBanners that cannot be presented', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/103004
    await tester.pumpWidget(const MaterialApp(home: Center()));

    final ScaffoldMessengerState scaffoldMessengerState = tester.state<ScaffoldMessengerState>(
      find.byType(ScaffoldMessenger),
    );
    expect(
      () {
        scaffoldMessengerState.showMaterialBanner(
          const MaterialBanner(content: Text('Banner'), actions: <Widget>[]),
        );
      },
      throwsA(
        isA<AssertionError>().having(
          (AssertionError error) => error.toString(),
          'description',
          contains(
            'ScaffoldMessenger.showMaterialBanner was called, but there are currently '
            'no descendant Scaffolds to present to.',
          ),
        ),
      ),
    );
  });

  testWidgets('Custom Margin respected', (WidgetTester tester) async {
    const EdgeInsets margin = EdgeInsets.all(30);
    await tester.pumpWidget(
      MaterialApp(
        home: MaterialBanner(
          margin: margin,
          content: const Text('I am a banner'),
          actions: <Widget>[TextButton(child: const Text('Action'), onPressed: () {})],
        ),
      ),
    );

    final Offset topLeft = tester.getTopLeft(
      find.descendant(of: find.byType(MaterialBanner), matching: find.byType(Material)).first,
    );

    /// Compare the offset of banner from top left
    expect(topLeft.dx, margin.left);
  });

  testWidgets('minActionBarHeight is respected', (WidgetTester tester) async {
    const double minActionBarHeight = 20.0;
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          appBar: AppBar(),
          body: const MaterialBanner(
            minActionBarHeight: minActionBarHeight,
            padding: EdgeInsets.zero,
            margin: EdgeInsets.zero,
            content: SizedBox.shrink(),
            actions: <Widget>[SizedBox.shrink()],
          ),
        ),
      ),
    );

    final Size size = tester.getSize(find.byType(MaterialBanner));
    expect(size.height, equals(minActionBarHeight));
  });

  testWidgets('minimumActionBarHeight is respected when presented by ScaffoldMessenger', (
    WidgetTester tester,
  ) async {
    const Key tapTarget = Key('tap-target');
    const double minActionBarHeight = 20.0;
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Builder(
            builder: (BuildContext context) {
              return GestureDetector(
                key: tapTarget,
                onTap: () {
                  ScaffoldMessenger.of(context).showMaterialBanner(
                    const MaterialBanner(
                      content: SizedBox.shrink(),
                      padding: EdgeInsets.zero,
                      margin: EdgeInsets.zero,
                      minActionBarHeight: minActionBarHeight,
                      actions: <Widget>[SizedBox.shrink()],
                    ),
                  );
                },
                behavior: HitTestBehavior.opaque,
                child: const SizedBox(height: 100.0, width: 100.0),
              );
            },
          ),
        ),
      ),
    );
    await tester.tap(find.byKey(tapTarget));
    await tester.pumpAndSettle();

    final Size materialBarSize = tester.getSize(find.byType(MaterialBanner));
    expect(materialBarSize.height, equals(minActionBarHeight));
  });
}

Material _getMaterialFromBanner(WidgetTester tester) {
  return tester.widget<Material>(
    find.descendant(of: find.byType(MaterialBanner), matching: find.byType(Material)).first,
  );
}

Material _getMaterialFromText(WidgetTester tester, String text) {
  return tester.widget<Material>(find.widgetWithText(Material, text).first);
}

RenderParagraph _getTextRenderObjectFromDialog(WidgetTester tester, String text) {
  return tester
          .element<StatelessElement>(
            find.descendant(of: find.byType(MaterialBanner), matching: find.text(text)),
          )
          .renderObject!
      as RenderParagraph;
}

bool _sizeAlmostEqual(Size a, Size b, {double maxDiff = 0.05}) {
  return (a.width - b.width).abs() <= maxDiff && (a.height - b.height).abs() <= maxDiff;
}
