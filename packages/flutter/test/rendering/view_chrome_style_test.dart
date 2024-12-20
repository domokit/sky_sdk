// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  group('SystemChrome - style', () {
    const double statusBarHeight = 25.0;
    const double navigationBarHeight = 54.0;
    const double deviceHeight = 960.0;
    const double deviceWidth = 480.0;
    const double devicePixelRatio = 2.0;

    void setupTestDevice(WidgetTester tester) {
      const FakeViewPadding padding = FakeViewPadding(
        top: statusBarHeight * devicePixelRatio,
        bottom: navigationBarHeight * devicePixelRatio,
      );

      addTearDown(tester.view.reset);
      tester.view
        ..viewPadding = padding
        ..padding = padding
        ..devicePixelRatio = devicePixelRatio
        ..physicalSize = const Size(
          deviceWidth * devicePixelRatio,
          deviceHeight * devicePixelRatio,
        );
    }

    tearDown(() async {
      SystemChrome.setSystemUIOverlayStyle(const SystemUiOverlayStyle());
      await pumpEventQueue();
    });

    group('status bar', () {
      testWidgets("statusBarColor isn't set for unannotated view", (WidgetTester tester) async {
        await tester.pumpWidget(const SizedBox.expand());
        await tester.pumpAndSettle();

        expect(SystemChrome.latestStyle?.statusBarColor, isNull);
      });

      testWidgets('statusBarColor is set for annotated view', (WidgetTester tester) async {
        setupTestDevice(tester);
        await tester.pumpWidget(
          const AnnotatedRegion<SystemUiOverlayStyle>(
            value: SystemUiOverlayStyle(statusBarColor: Colors.blue),
            child: SizedBox.expand(),
          ),
        );
        await tester.pumpAndSettle();

        expect(SystemChrome.latestStyle?.statusBarColor, Colors.blue);
      }, variant: TargetPlatformVariant.mobile());

      testWidgets(
        "statusBarColor isn't set when view covers less than half of the system status bar",
        (WidgetTester tester) async {
          setupTestDevice(tester);
          const double lessThanHalfOfTheStatusBarHeight = statusBarHeight / 2.0 - 1;
          await tester.pumpWidget(
            const Align(
              alignment: Alignment.topCenter,
              child: AnnotatedRegion<SystemUiOverlayStyle>(
                value: SystemUiOverlayStyle(statusBarColor: Colors.blue),
                child: SizedBox(width: 100, height: lessThanHalfOfTheStatusBarHeight),
              ),
            ),
          );
          await tester.pumpAndSettle();

          expect(SystemChrome.latestStyle?.statusBarColor, isNull);
        },
        variant: TargetPlatformVariant.mobile(),
      );

      testWidgets(
        'statusBarColor is set when view covers more than half of tye system status bar',
        (WidgetTester tester) async {
          setupTestDevice(tester);
          const double moreThanHalfOfTheStatusBarHeight = statusBarHeight / 2.0 + 1;
          await tester.pumpWidget(
            const Align(
              alignment: Alignment.topCenter,
              child: AnnotatedRegion<SystemUiOverlayStyle>(
                value: SystemUiOverlayStyle(statusBarColor: Colors.blue),
                child: SizedBox(width: 100, height: moreThanHalfOfTheStatusBarHeight),
              ),
            ),
          );
          await tester.pumpAndSettle();

          expect(SystemChrome.latestStyle?.statusBarColor, Colors.blue);
        },
        variant: TargetPlatformVariant.mobile(),
      );
    });

    group('navigation color (Android only)', () {
      testWidgets(
        "systemNavigationBarColor isn't set for non Android device",
        (WidgetTester tester) async {
          setupTestDevice(tester);
          await tester.pumpWidget(
            const AnnotatedRegion<SystemUiOverlayStyle>(
              value: SystemUiOverlayStyle(systemNavigationBarColor: Colors.blue),
              child: SizedBox.expand(),
            ),
          );
          await tester.pumpAndSettle();

          expect(SystemChrome.latestStyle?.systemNavigationBarColor, isNull);
        },
        variant: TargetPlatformVariant.only(TargetPlatform.iOS),
      );

      testWidgets(
        "systemNavigationBarColor isn't set for unannotated view",
        (WidgetTester tester) async {
          await tester.pumpWidget(const SizedBox.expand());
          await tester.pumpAndSettle();

          expect(SystemChrome.latestStyle?.systemNavigationBarColor, isNull);
        },
        variant: TargetPlatformVariant.only(TargetPlatform.android),
      );

      testWidgets(
        'systemNavigationBarColor is set for annotated view',
        (WidgetTester tester) async {
          setupTestDevice(tester);
          await tester.pumpWidget(
            const AnnotatedRegion<SystemUiOverlayStyle>(
              value: SystemUiOverlayStyle(systemNavigationBarColor: Colors.blue),
              child: SizedBox.expand(),
            ),
          );
          await tester.pumpAndSettle();

          expect(SystemChrome.latestStyle?.systemNavigationBarColor, Colors.blue);
        },
        variant: TargetPlatformVariant.only(TargetPlatform.android),
      );

      testWidgets(
        "systemNavigationBarColor isn't set when view covers less than half of navigation bar",
        (WidgetTester tester) async {
          setupTestDevice(tester);
          const double lessThanHalfOfTheNavigationBarHeight = navigationBarHeight / 2.0 - 1;
          await tester.pumpWidget(
            const Align(
              alignment: Alignment.bottomCenter,
              child: AnnotatedRegion<SystemUiOverlayStyle>(
                value: SystemUiOverlayStyle(systemNavigationBarColor: Colors.blue),
                child: SizedBox(width: 100, height: lessThanHalfOfTheNavigationBarHeight),
              ),
            ),
          );
          await tester.pumpAndSettle();

          expect(SystemChrome.latestStyle?.systemNavigationBarColor, isNull);
        },
        variant: TargetPlatformVariant.only(TargetPlatform.android),
      );

      testWidgets(
        'systemNavigationBarColor is set when view covers more than half of navigation bar',
        (WidgetTester tester) async {
          setupTestDevice(tester);
          const double moreThanHalfOfTheNavigationBarHeight = navigationBarHeight / 2.0 + 1;
          await tester.pumpWidget(
            const Align(
              alignment: Alignment.bottomCenter,
              child: AnnotatedRegion<SystemUiOverlayStyle>(
                value: SystemUiOverlayStyle(systemNavigationBarColor: Colors.blue),
                child: SizedBox(width: 100, height: moreThanHalfOfTheNavigationBarHeight),
              ),
            ),
          );
          await tester.pumpAndSettle();

          expect(SystemChrome.latestStyle?.systemNavigationBarColor, Colors.blue);
        },
        variant: TargetPlatformVariant.only(TargetPlatform.android),
      );
    });

    testWidgets(
      'Top AnnotatedRegion provides status bar overlay style and bottom AnnotatedRegion provides navigation bar overlay style',
      (WidgetTester tester) async {
        setupTestDevice(tester);
        await tester.pumpWidget(
          const Column(
            children: <Widget>[
              Expanded(
                child: AnnotatedRegion<SystemUiOverlayStyle>(
                  value: SystemUiOverlayStyle(
                    systemNavigationBarColor: Colors.blue,
                    statusBarColor: Colors.blue,
                  ),
                  child: SizedBox.expand(),
                ),
              ),
              Expanded(
                child: AnnotatedRegion<SystemUiOverlayStyle>(
                  value: SystemUiOverlayStyle(
                    systemNavigationBarColor: Colors.green,
                    statusBarColor: Colors.green,
                  ),
                  child: SizedBox.expand(),
                ),
              ),
            ],
          ),
        );
        await tester.pumpAndSettle();

        expect(SystemChrome.latestStyle?.statusBarColor, Colors.blue);
        expect(SystemChrome.latestStyle?.systemNavigationBarColor, Colors.green);
      },
      variant: TargetPlatformVariant.only(TargetPlatform.android),
    );

    testWidgets(
      'Top only AnnotatedRegion provides status bar and navigation bar style properties',
      (WidgetTester tester) async {
        setupTestDevice(tester);
        await tester.pumpWidget(
          const Column(
            children: <Widget>[
              Expanded(
                child: AnnotatedRegion<SystemUiOverlayStyle>(
                  value: SystemUiOverlayStyle(
                    systemNavigationBarColor: Colors.blue,
                    statusBarColor: Colors.blue,
                  ),
                  child: SizedBox.expand(),
                ),
              ),
              Expanded(child: SizedBox.expand()),
            ],
          ),
        );
        await tester.pumpAndSettle();

        expect(SystemChrome.latestStyle?.statusBarColor, Colors.blue);
        expect(SystemChrome.latestStyle?.systemNavigationBarColor, Colors.blue);
      },
      variant: TargetPlatformVariant.only(TargetPlatform.android),
    );

    testWidgets(
      'Bottom only AnnotatedRegion provides status bar and navigation bar style properties',
      (WidgetTester tester) async {
        setupTestDevice(tester);
        await tester.pumpWidget(
          const Column(
            children: <Widget>[
              Expanded(child: SizedBox.expand()),
              Expanded(
                child: AnnotatedRegion<SystemUiOverlayStyle>(
                  value: SystemUiOverlayStyle(
                    systemNavigationBarColor: Colors.green,
                    statusBarColor: Colors.green,
                  ),
                  child: SizedBox.expand(),
                ),
              ),
            ],
          ),
        );
        await tester.pumpAndSettle();

        expect(SystemChrome.latestStyle?.statusBarColor, Colors.green);
        expect(SystemChrome.latestStyle?.systemNavigationBarColor, Colors.green);
      },
      variant: TargetPlatformVariant.only(TargetPlatform.android),
    );
  });
}
