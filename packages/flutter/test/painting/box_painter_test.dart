// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This file is run as part of a reduced test set in CI on Mac and Windows
// machines.
@Tags(<String>['reduced-test-set'])
library;

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  tearDown(() {
    debugDisableShadows = true;
  });

  test('BorderSide control test', () {
    const BorderSide side1 = BorderSide();
    final BorderSide side2 = side1.copyWith(
      color: const Color(0xFF00FFFF),
      width: 2.0,
      style: BorderStyle.solid,
    );

    expect(side1, hasOneLineDescription);
    expect(side1.hashCode, isNot(equals(side2.hashCode)));

    expect(side2.color, equals(const Color(0xFF00FFFF)));
    expect(side2.width, equals(2.0));
    expect(side2.style, equals(BorderStyle.solid));

    expect(BorderSide.lerp(side1, side2, 0.0), equals(side1));
    expect(BorderSide.lerp(side1, side2, 1.0), equals(side2));
    expect(
      BorderSide.lerp(side1, side2, 0.5),
      equals(
        BorderSide(
          color: Color.lerp(const Color(0xFF000000), const Color(0xFF00FFFF), 0.5)!,
          width: 1.5,
        ),
      ),
    );

    final BorderSide side3 = side2.copyWith(style: BorderStyle.none);
    BorderSide interpolated = BorderSide.lerp(side2, side3, 0.2);
    expect(interpolated.style, equals(BorderStyle.solid));
    expect(interpolated.color, equals(side2.color.withOpacity(0.8)));

    interpolated = BorderSide.lerp(side3, side2, 0.2);
    expect(interpolated.style, equals(BorderStyle.solid));
    expect(interpolated.color, equals(side2.color.withOpacity(0.2)));
  });

  test('BorderSide toString test', () {
    const BorderSide side1 = BorderSide();
    final BorderSide side2 = side1.copyWith(
      color: const Color(0xFF00FFFF),
      width: 2.0,
      style: BorderStyle.solid,
    );

    expect(side1.toString(), equals('BorderSide'));
    expect(side2.toString(), equals('BorderSide(color: ${const Color(0xff00ffff)}, width: 2.0)'));
  });

  test('Border control test', () {
    final Border border1 = Border.all(width: 4.0);
    final Border border2 = Border.lerp(null, border1, 0.25)!;
    final Border border3 = Border.lerp(border1, null, 0.25)!;

    expect(border1, hasOneLineDescription);
    expect(border1.hashCode, isNot(equals(border2.hashCode)));

    expect(border2.top.width, equals(1.0));
    expect(border3.bottom.width, equals(3.0));

    final Border border4 = Border.lerp(border2, border3, 0.5)!;
    expect(border4.left.width, equals(2.0));
  });

  test('Border toString test', () {
    expect(Border.all(width: 4.0).toString(), equals('Border.all(BorderSide(width: 4.0))'));
    expect(
      const Border(
        top: BorderSide(width: 3.0),
        right: BorderSide(width: 3.0),
        bottom: BorderSide(width: 3.0),
        left: BorderSide(width: 3.0),
      ).toString(),
      equals('Border.all(BorderSide(width: 3.0))'),
    );
  });

  test('BoxShadow control test', () {
    const BoxShadow shadow1 = BoxShadow(blurRadius: 4.0);
    final BoxShadow shadow2 = BoxShadow.lerp(null, shadow1, 0.25)!;
    final BoxShadow shadow3 = BoxShadow.lerp(shadow1, null, 0.25)!;

    expect(shadow1, hasOneLineDescription);
    expect(shadow1.hashCode, isNot(equals(shadow2.hashCode)));
    expect(shadow1, equals(const BoxShadow(blurRadius: 4.0)));

    expect(shadow2.blurRadius, equals(1.0));
    expect(shadow3.blurRadius, equals(3.0));

    final BoxShadow shadow4 = BoxShadow.lerp(shadow2, shadow3, 0.5)!;
    expect(shadow4.blurRadius, equals(2.0));

    List<BoxShadow> shadowList =
        BoxShadow.lerpList(<BoxShadow>[shadow2, shadow1], <BoxShadow>[shadow3], 0.5)!;
    expect(shadowList, equals(<BoxShadow>[shadow4, shadow1.scale(0.5)]));
    shadowList = BoxShadow.lerpList(<BoxShadow>[shadow2], <BoxShadow>[shadow3, shadow1], 0.5)!;
    expect(shadowList, equals(<BoxShadow>[shadow4, shadow1.scale(0.5)]));
  });

  test('BoxShadow.lerp identical a,b', () {
    expect(BoxShadow.lerp(null, null, 0), null);
    const BoxShadow border = BoxShadow();
    expect(identical(BoxShadow.lerp(border, border, 0.5), border), true);
  });

  test('BoxShadowList.lerp identical a,b', () {
    expect(BoxShadow.lerpList(null, null, 0), null);
    const List<BoxShadow> border = <BoxShadow>[BoxShadow()];
    expect(identical(BoxShadow.lerpList(border, border, 0.5), border), true);
  });

  test('BoxShadow BlurStyle test', () {
    const BoxShadow shadow1 = BoxShadow(blurRadius: 4.0);
    const BoxShadow shadow2 = BoxShadow(blurRadius: 4.0, blurStyle: BlurStyle.outer);
    final BoxShadow shadow3 = BoxShadow.lerp(shadow1, null, 0.25)!;
    final BoxShadow shadow4 = BoxShadow.lerp(null, shadow1, 0.25)!;
    final BoxShadow shadow5 = BoxShadow.lerp(shadow1, shadow2, 0.25)!;
    final BoxShadow shadow6 =
        BoxShadow.lerp(const BoxShadow(blurStyle: BlurStyle.solid), shadow2, 0.25)!;

    expect(shadow1.blurStyle, equals(BlurStyle.normal));
    expect(shadow2.blurStyle, equals(BlurStyle.outer));
    expect(shadow3.blurStyle, equals(BlurStyle.normal));
    expect(shadow4.blurStyle, equals(BlurStyle.normal));
    expect(shadow5.blurStyle, equals(BlurStyle.outer));
    expect(shadow6.blurStyle, equals(BlurStyle.solid));

    List<BoxShadow> shadowList =
        BoxShadow.lerpList(<BoxShadow>[shadow2, shadow1], <BoxShadow>[shadow3], 0.5)!;
    expect(shadowList[0].blurStyle, equals(BlurStyle.outer));
    expect(shadowList[1].blurStyle, equals(BlurStyle.normal));

    shadowList = BoxShadow.lerpList(<BoxShadow>[shadow6], <BoxShadow>[shadow3, shadow1], 0.5)!;
    expect(shadowList[0].blurStyle, equals(BlurStyle.solid));
    expect(shadowList[1].blurStyle, equals(BlurStyle.normal));

    shadowList = BoxShadow.lerpList(<BoxShadow>[shadow3], <BoxShadow>[shadow6, shadow1], 0.5)!;
    expect(shadowList[0].blurStyle, equals(BlurStyle.solid));
    expect(shadowList[1].blurStyle, equals(BlurStyle.normal));

    shadowList = BoxShadow.lerpList(<BoxShadow>[shadow3], <BoxShadow>[shadow2, shadow1], 0.5)!;
    expect(shadowList[0].blurStyle, equals(BlurStyle.outer));
    expect(shadowList[1].blurStyle, equals(BlurStyle.normal));
  });

  test('BoxShadow toString test', () {
    expect(
      const BoxShadow(blurRadius: 4.0).toString(),
      equals('BoxShadow(${const Color(0xff000000)}, Offset(0.0, 0.0), 4.0, 0.0, BlurStyle.normal)'),
    );
    expect(
      const BoxShadow(blurRadius: 4.0, blurStyle: BlurStyle.solid).toString(),
      equals('BoxShadow(${const Color(0xff000000)}, Offset(0.0, 0.0), 4.0, 0.0, BlurStyle.solid)'),
    );
  });

  testWidgets('BoxShadow BoxStyle.solid', (WidgetTester tester) async {
    final Key key = UniqueKey();
    debugDisableShadows = false;
    await tester.pumpWidget(
      Center(
        child: RepaintBoundary(
          key: key,
          child: Container(
            color: Colors.white,
            width: 50,
            height: 50,
            child: Center(
              child: Container(
                decoration: const BoxDecoration(
                  boxShadow: <BoxShadow>[BoxShadow(blurRadius: 3.0, blurStyle: BlurStyle.solid)],
                ),
                width: 10,
                height: 10,
              ),
            ),
          ),
        ),
      ),
    );

    await expectLater(find.byKey(key), matchesGoldenFile('boxShadow.boxStyle.solid.0.0.png'));
    debugDisableShadows = true;
  });

  testWidgets('BoxShadow BoxStyle.outer', (WidgetTester tester) async {
    final Key key = UniqueKey();
    debugDisableShadows = false;
    await tester.pumpWidget(
      Center(
        child: RepaintBoundary(
          key: key,
          child: Container(
            color: Colors.white,
            width: 50,
            height: 50,
            child: Center(
              child: Container(
                decoration: const BoxDecoration(
                  boxShadow: <BoxShadow>[BoxShadow(blurRadius: 8.0, blurStyle: BlurStyle.outer)],
                ),
                width: 20,
                height: 20,
              ),
            ),
          ),
        ),
      ),
    );

    await expectLater(find.byKey(key), matchesGoldenFile('boxShadow.boxStyle.outer.0.0.png'));
    debugDisableShadows = true;
  });

  testWidgets('BoxShadow BoxStyle.inner', (WidgetTester tester) async {
    final Key key = UniqueKey();
    debugDisableShadows = false;
    await tester.pumpWidget(
      Center(
        child: RepaintBoundary(
          key: key,
          child: Container(
            color: Colors.white,
            width: 50,
            height: 50,
            child: Center(
              child: Container(
                decoration: const BoxDecoration(
                  boxShadow: <BoxShadow>[BoxShadow(blurRadius: 4.0, blurStyle: BlurStyle.inner)],
                ),
                width: 20,
                height: 20,
              ),
            ),
          ),
        ),
      ),
    );

    await expectLater(find.byKey(key), matchesGoldenFile('boxShadow.boxStyle.inner.0.0.png'));
    debugDisableShadows = true;
  });

  testWidgets('BoxShadow BoxStyle.normal', (WidgetTester tester) async {
    final Key key = UniqueKey();
    debugDisableShadows = false;
    await tester.pumpWidget(
      Center(
        child: RepaintBoundary(
          key: key,
          child: Container(
            color: Colors.white,
            width: 50,
            height: 50,
            child: Center(
              child: Container(
                decoration: const BoxDecoration(boxShadow: <BoxShadow>[BoxShadow(blurRadius: 4.0)]),
                width: 20,
                height: 20,
              ),
            ),
          ),
        ),
      ),
    );

    await expectLater(find.byKey(key), matchesGoldenFile('boxShadow.boxStyle.normal.0.0.png'));
    debugDisableShadows = true;
  });

  testWidgets('BoxShadow BoxStyle.normal.wide_radius', (WidgetTester tester) async {
    final Key key = UniqueKey();
    debugDisableShadows = false;
    await tester.pumpWidget(
      Center(
        child: RepaintBoundary(
          key: key,
          child: Container(
            color: Colors.amber,
            width: 128,
            height: 128,
            child: Center(
              child: Container(
                decoration: const BoxDecoration(
                  color: Colors.black,
                  boxShadow: <BoxShadow>[
                    BoxShadow(
                      blurRadius: 16.0,
                      offset: Offset(4, 4),
                      color: Colors.green,
                      spreadRadius: 2,
                    ),
                  ],
                ),
                width: 64,
                height: 64,
              ),
            ),
          ),
        ),
      ),
    );

    await expectLater(
      find.byKey(key),
      matchesGoldenFile('boxShadow.boxStyle.normal.wide_radius.0.0.png'),
    );
    debugDisableShadows = true;
  });

  testWidgets('BoxShadow BoxStyle.outer.wide_radius', (WidgetTester tester) async {
    final Key key = UniqueKey();
    debugDisableShadows = false;
    await tester.pumpWidget(
      Center(
        child: RepaintBoundary(
          key: key,
          child: Container(
            color: Colors.amber,
            width: 128,
            height: 128,
            child: Center(
              child: Container(
                decoration: const BoxDecoration(
                  color: Colors.black,
                  boxShadow: <BoxShadow>[
                    BoxShadow(
                      blurRadius: 16.0,
                      offset: Offset(4, 4),
                      blurStyle: BlurStyle.outer,
                      color: Colors.red,
                      spreadRadius: 2,
                    ),
                  ],
                ),
                width: 64,
                height: 64,
              ),
            ),
          ),
        ),
      ),
    );

    await expectLater(
      find.byKey(key),
      matchesGoldenFile('boxShadow.boxStyle.outer.wide_radius.0.0.png'),
    );
    debugDisableShadows = true;
  });

  testWidgets('BoxShadow BoxStyle.solid.wide_radius', (WidgetTester tester) async {
    final Key key = UniqueKey();
    debugDisableShadows = false;
    await tester.pumpWidget(
      Center(
        child: RepaintBoundary(
          key: key,
          child: Container(
            color: Colors.grey,
            width: 128,
            height: 128,
            child: Center(
              child: Container(
                decoration: const BoxDecoration(
                  color: Colors.black,
                  boxShadow: <BoxShadow>[
                    BoxShadow(
                      blurRadius: 16.0,
                      offset: Offset(4, 4),
                      blurStyle: BlurStyle.solid,
                      color: Colors.purple,
                      spreadRadius: 2,
                    ),
                  ],
                ),
                width: 64,
                height: 64,
              ),
            ),
          ),
        ),
      ),
    );
    await expectLater(
      find.byKey(key),
      matchesGoldenFile('boxShadow.boxStyle.solid.wide_radius.0.0.png'),
    );
    debugDisableShadows = true;
  });

  testWidgets('BoxShadow BoxStyle.inner.wide_radius', (WidgetTester tester) async {
    final Key key = UniqueKey();
    debugDisableShadows = false;
    await tester.pumpWidget(
      Center(
        child: RepaintBoundary(
          key: key,
          child: Container(
            color: Colors.green,
            width: 128,
            height: 128,
            child: Center(
              child: Container(
                decoration: const BoxDecoration(
                  color: Colors.black,
                  boxShadow: <BoxShadow>[
                    BoxShadow(
                      blurRadius: 16.0,
                      offset: Offset(4, 4),
                      blurStyle: BlurStyle.inner,
                      color: Colors.amber,
                      spreadRadius: 2,
                    ),
                  ],
                ),
                width: 64,
                height: 64,
              ),
            ),
          ),
        ),
      ),
    );
    await expectLater(
      find.byKey(key),
      matchesGoldenFile('boxShadow.boxStyle.inner.wide_radius.0.0.png'),
    );
    debugDisableShadows = true;
  });
}
