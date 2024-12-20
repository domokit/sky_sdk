// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math' as math;

import 'package:flutter/rendering.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

import 'two_dimensional_utils.dart';

Finder findKey(int i) => find.byKey(ValueKey<int>(i), skipOffstage: false);

Widget buildSingleChildScrollView(Axis scrollDirection, {bool reverse = false}) {
  return Directionality(
    textDirection: TextDirection.ltr,
    child: Center(
      child: SizedBox(
        width: 600.0,
        height: 400.0,
        child: SingleChildScrollView(
          scrollDirection: scrollDirection,
          reverse: reverse,
          child: ListBody(
            mainAxis: scrollDirection,
            children: const <Widget>[
              SizedBox(key: ValueKey<int>(0), width: 200.0, height: 200.0),
              SizedBox(key: ValueKey<int>(1), width: 200.0, height: 200.0),
              SizedBox(key: ValueKey<int>(2), width: 200.0, height: 200.0),
              SizedBox(key: ValueKey<int>(3), width: 200.0, height: 200.0),
              SizedBox(key: ValueKey<int>(4), width: 200.0, height: 200.0),
              SizedBox(key: ValueKey<int>(5), width: 200.0, height: 200.0),
              SizedBox(key: ValueKey<int>(6), width: 200.0, height: 200.0),
            ],
          ),
        ),
      ),
    ),
  );
}

Widget buildListView(Axis scrollDirection, {bool reverse = false, bool shrinkWrap = false}) {
  return Directionality(
    textDirection: TextDirection.ltr,
    child: Center(
      child: SizedBox(
        width: 600.0,
        height: 400.0,
        child: ListView(
          scrollDirection: scrollDirection,
          reverse: reverse,
          addSemanticIndexes: false,
          shrinkWrap: shrinkWrap,
          children: const <Widget>[
            SizedBox(key: ValueKey<int>(0), width: 200.0, height: 200.0),
            SizedBox(key: ValueKey<int>(1), width: 200.0, height: 200.0),
            SizedBox(key: ValueKey<int>(2), width: 200.0, height: 200.0),
            SizedBox(key: ValueKey<int>(3), width: 200.0, height: 200.0),
            SizedBox(key: ValueKey<int>(4), width: 200.0, height: 200.0),
            SizedBox(key: ValueKey<int>(5), width: 200.0, height: 200.0),
            SizedBox(key: ValueKey<int>(6), width: 200.0, height: 200.0),
          ],
        ),
      ),
    ),
  );
}

void main() {
  group('SingleChildScrollView', () {
    testWidgets('SingleChildScrollView ensureVisible Axis.vertical', (WidgetTester tester) async {
      BuildContext findContext(int i) => tester.element(findKey(i));

      await tester.pumpWidget(buildSingleChildScrollView(Axis.vertical));

      Scrollable.ensureVisible(findContext(3));
      await tester.pump();
      expect(tester.getTopLeft(findKey(3)).dy, equals(100.0));

      Scrollable.ensureVisible(findContext(6));
      await tester.pump();
      expect(tester.getTopLeft(findKey(6)).dy, equals(300.0));

      Scrollable.ensureVisible(findContext(4), alignment: 1.0);
      await tester.pump();
      expect(tester.getBottomRight(findKey(4)).dy, equals(500.0));

      Scrollable.ensureVisible(findContext(0), alignment: 1.0);
      await tester.pump();
      expect(tester.getTopLeft(findKey(0)).dy, equals(100.0));

      Scrollable.ensureVisible(findContext(3), duration: const Duration(seconds: 1));
      await tester.pump();
      await tester.pump(const Duration(milliseconds: 1020));
      expect(tester.getTopLeft(findKey(3)).dy, equals(100.0));
    });

    testWidgets('SingleChildScrollView ensureVisible Axis.horizontal', (WidgetTester tester) async {
      BuildContext findContext(int i) => tester.element(findKey(i));

      await tester.pumpWidget(buildSingleChildScrollView(Axis.horizontal));

      Scrollable.ensureVisible(findContext(3));
      await tester.pump();
      expect(tester.getTopLeft(findKey(3)).dx, equals(100.0));

      Scrollable.ensureVisible(findContext(6));
      await tester.pump();
      expect(tester.getTopLeft(findKey(6)).dx, equals(500.0));

      Scrollable.ensureVisible(findContext(4), alignment: 1.0);
      await tester.pump();
      expect(tester.getBottomRight(findKey(4)).dx, equals(700.0));

      Scrollable.ensureVisible(findContext(0), alignment: 1.0);
      await tester.pump();
      expect(tester.getTopLeft(findKey(0)).dx, equals(100.0));

      Scrollable.ensureVisible(findContext(3), duration: const Duration(seconds: 1));
      await tester.pump();
      await tester.pump(const Duration(milliseconds: 1020));
      expect(tester.getTopLeft(findKey(3)).dx, equals(100.0));
    });

    testWidgets('SingleChildScrollView ensureVisible Axis.vertical reverse', (
      WidgetTester tester,
    ) async {
      BuildContext findContext(int i) => tester.element(findKey(i));

      await tester.pumpWidget(buildSingleChildScrollView(Axis.vertical, reverse: true));

      Scrollable.ensureVisible(findContext(3));
      await tester.pump();
      expect(tester.getBottomRight(findKey(3)).dy, equals(500.0));

      Scrollable.ensureVisible(findContext(0));
      await tester.pump();
      expect(tester.getBottomRight(findKey(0)).dy, equals(300.0));

      Scrollable.ensureVisible(findContext(2), alignment: 1.0);
      await tester.pump();
      expect(tester.getTopLeft(findKey(2)).dy, equals(100.0));

      Scrollable.ensureVisible(findContext(6), alignment: 1.0);
      await tester.pump();
      expect(tester.getBottomRight(findKey(6)).dy, equals(500.0));

      Scrollable.ensureVisible(findContext(3), duration: const Duration(seconds: 1));
      await tester.pump();
      await tester.pump(const Duration(milliseconds: 1020));
      expect(tester.getBottomRight(findKey(3)).dy, equals(500.0));

      // Regression test for https://github.com/flutter/flutter/issues/128749
      // Reset to zero position.
      tester.state<ScrollableState>(find.byType(Scrollable)).position.jumpTo(0.0);
      await tester.pump();
      // 4 is not currently visible as the SingleChildScrollView is contained
      // within a centered SizedBox.
      expect(tester.getBottomLeft(findKey(4)).dy, equals(100.0));
      expect(tester.getBottomLeft(findKey(6)).dy, equals(500.0));
      Scrollable.ensureVisible(
        findContext(6),
        alignmentPolicy: ScrollPositionAlignmentPolicy.keepVisibleAtStart,
      );
      await tester.pump();
      Scrollable.ensureVisible(
        findContext(5),
        alignmentPolicy: ScrollPositionAlignmentPolicy.keepVisibleAtStart,
      );
      await tester.pump();
      // 5 and 6 are already visible beyond the top edge, so no change.
      expect(tester.getBottomLeft(findKey(4)).dy, equals(100.0));
      expect(tester.getBottomLeft(findKey(6)).dy, equals(500.0));
      Scrollable.ensureVisible(
        findContext(4),
        alignmentPolicy: ScrollPositionAlignmentPolicy.keepVisibleAtStart,
      );
      await tester.pump();
      // Since it is reversed, 4 should have come into view at the top
      // edge of the scrollable, matching the alignment expectation.
      expect(tester.getBottomLeft(findKey(4)).dy, equals(300.0));
      expect(tester.getBottomLeft(findKey(6)).dy, equals(700.0));

      // Bring 6 back into view at the trailing edge, checking the other
      // alignment.
      Scrollable.ensureVisible(
        findContext(6),
        alignmentPolicy: ScrollPositionAlignmentPolicy.keepVisibleAtEnd,
      );
      await tester.pump();
      expect(tester.getBottomLeft(findKey(4)).dy, equals(100.0));
      expect(tester.getBottomLeft(findKey(6)).dy, equals(500.0));
    });

    testWidgets('SingleChildScrollView ensureVisible Axis.horizontal reverse', (
      WidgetTester tester,
    ) async {
      BuildContext findContext(int i) => tester.element(findKey(i));

      await tester.pumpWidget(buildSingleChildScrollView(Axis.horizontal, reverse: true));

      Scrollable.ensureVisible(findContext(3));
      await tester.pump();
      expect(tester.getBottomRight(findKey(3)).dx, equals(700.0));

      Scrollable.ensureVisible(findContext(0));
      await tester.pump();
      expect(tester.getBottomRight(findKey(0)).dx, equals(300.0));

      Scrollable.ensureVisible(findContext(2), alignment: 1.0);
      await tester.pump();
      expect(tester.getTopLeft(findKey(2)).dx, equals(100.0));

      Scrollable.ensureVisible(findContext(6), alignment: 1.0);
      await tester.pump();
      expect(tester.getBottomRight(findKey(6)).dx, equals(700.0));

      Scrollable.ensureVisible(findContext(3), duration: const Duration(seconds: 1));
      await tester.pump();
      await tester.pump(const Duration(milliseconds: 1020));
      expect(tester.getBottomRight(findKey(3)).dx, equals(700.0));

      // Regression test for https://github.com/flutter/flutter/issues/128749
      // Reset to zero position.
      tester.state<ScrollableState>(find.byType(Scrollable)).position.jumpTo(0.0);
      await tester.pump();
      // 4 is not currently visible as the SingleChildScrollView is contained
      // within a centered SizedBox.
      expect(tester.getBottomLeft(findKey(3)).dx, equals(-100.0));
      expect(tester.getBottomLeft(findKey(6)).dx, equals(500.0));
      Scrollable.ensureVisible(
        findContext(6),
        alignmentPolicy: ScrollPositionAlignmentPolicy.keepVisibleAtStart,
      );
      await tester.pump();
      Scrollable.ensureVisible(
        findContext(5),
        alignmentPolicy: ScrollPositionAlignmentPolicy.keepVisibleAtStart,
      );
      await tester.pump();
      Scrollable.ensureVisible(
        findContext(4),
        alignmentPolicy: ScrollPositionAlignmentPolicy.keepVisibleAtStart,
      );
      await tester.pump();
      // 4, 5 and 6 are already visible beyond the left edge, so no change.
      expect(tester.getBottomLeft(findKey(3)).dx, equals(-100.0));
      expect(tester.getBottomLeft(findKey(6)).dx, equals(500.0));
      Scrollable.ensureVisible(
        findContext(3),
        alignmentPolicy: ScrollPositionAlignmentPolicy.keepVisibleAtStart,
      );
      await tester.pump();
      // Since it is reversed, 3 should have come into view at the leading
      // edge of the scrollable, matching the alignment expectation.
      expect(tester.getBottomLeft(findKey(3)).dx, equals(100.0));
      expect(tester.getBottomLeft(findKey(6)).dx, equals(700.0));

      // Bring 6 back into view at the trailing edge, checking the other
      // alignment.
      Scrollable.ensureVisible(
        findContext(6),
        alignmentPolicy: ScrollPositionAlignmentPolicy.keepVisibleAtEnd,
      );
      await tester.pump();
      expect(tester.getBottomLeft(findKey(3)).dx, equals(-100.0));
      expect(tester.getBottomLeft(findKey(6)).dx, equals(500.0));
    });

    testWidgets('SingleChildScrollView ensureVisible rotated child', (WidgetTester tester) async {
      BuildContext findContext(int i) => tester.element(findKey(i));

      await tester.pumpWidget(
        Center(
          child: SizedBox(
            width: 600.0,
            height: 400.0,
            child: SingleChildScrollView(
              child: ListBody(
                children: <Widget>[
                  const SizedBox(height: 200.0),
                  const SizedBox(height: 200.0),
                  const SizedBox(height: 200.0),
                  SizedBox(
                    height: 200.0,
                    child: Center(
                      child: Transform(
                        transform: Matrix4.rotationZ(math.pi),
                        child: Container(
                          key: const ValueKey<int>(0),
                          width: 100.0,
                          height: 100.0,
                          color: const Color(0xFFFFFFFF),
                        ),
                      ),
                    ),
                  ),
                  const SizedBox(height: 200.0),
                  const SizedBox(height: 200.0),
                  const SizedBox(height: 200.0),
                ],
              ),
            ),
          ),
        ),
      );

      Scrollable.ensureVisible(findContext(0));
      await tester.pump();
      expect(tester.getBottomRight(findKey(0)).dy, moreOrLessEquals(100.0, epsilon: 0.1));

      Scrollable.ensureVisible(findContext(0), alignment: 1.0);
      await tester.pump();
      expect(tester.getTopLeft(findKey(0)).dy, moreOrLessEquals(500.0, epsilon: 0.1));
    });

    testWidgets('Nested SingleChildScrollView ensureVisible behavior test', (
      WidgetTester tester,
    ) async {
      // Regressing test for https://github.com/flutter/flutter/issues/65100
      Finder findKey(String coordinate) => find.byKey(ValueKey<String>(coordinate));
      BuildContext findContext(String coordinate) => tester.element(findKey(coordinate));
      final List<Row> rows = List<Row>.generate(
        7,
        (int y) => Row(
          children: List<SizedBox>.generate(
            7,
            (int x) => SizedBox(key: ValueKey<String>('$x, $y'), width: 200.0, height: 200.0),
          ),
        ),
      );

      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: Center(
            child: SizedBox(
              width: 600.0,
              height: 400.0,
              child: SingleChildScrollView(
                scrollDirection: Axis.horizontal,
                child: SingleChildScrollView(child: Column(children: rows)),
              ),
            ),
          ),
        ),
      );

      //      Items: 7 * 7 Container(width: 200.0, height: 200.0)
      //      viewport: Size(width: 600.0, height: 400.0)
      //
      //               0                       600
      //                 +----------------------+
      //                 |0,0    |1,0    |2,0   |
      //                 |       |       |      |
      //                 +----------------------+
      //                 |0,1    |1,1    |2,1   |
      //                 |       |       |      |
      //             400 +----------------------+

      Scrollable.ensureVisible(findContext('0, 0'));
      await tester.pump();
      expect(tester.getTopLeft(findKey('0, 0')), const Offset(100.0, 100.0));

      Scrollable.ensureVisible(findContext('3, 0'));
      await tester.pump();
      expect(tester.getTopLeft(findKey('3, 0')), const Offset(100.0, 100.0));

      Scrollable.ensureVisible(findContext('3, 0'), alignment: 0.5);
      await tester.pump();
      expect(tester.getTopLeft(findKey('3, 0')), const Offset(300.0, 100.0));

      Scrollable.ensureVisible(findContext('6, 0'));
      await tester.pump();
      expect(tester.getTopLeft(findKey('6, 0')), const Offset(500.0, 100.0));

      Scrollable.ensureVisible(findContext('0, 2'));
      await tester.pump();
      expect(tester.getTopLeft(findKey('0, 2')), const Offset(100.0, 100.0));

      Scrollable.ensureVisible(findContext('3, 2'));
      await tester.pump();
      expect(tester.getTopLeft(findKey('3, 2')), const Offset(100.0, 100.0));

      // It should be at the center of the screen.
      Scrollable.ensureVisible(findContext('3, 2'), alignment: 0.5);
      await tester.pump();
      expect(tester.getTopLeft(findKey('3, 2')), const Offset(300.0, 200.0));
    });
  });

  group('ListView', () {
    testWidgets('ListView ensureVisible Axis.vertical', (WidgetTester tester) async {
      BuildContext findContext(int i) => tester.element(findKey(i));
      Future<void> prepare(double offset) async {
        tester.state<ScrollableState>(find.byType(Scrollable)).position.jumpTo(offset);
        await tester.pump();
      }

      await tester.pumpWidget(buildListView(Axis.vertical));

      await prepare(480.0);
      Scrollable.ensureVisible(findContext(3));
      await tester.pump();
      expect(tester.getTopLeft(findKey(3)).dy, equals(100.0));

      await prepare(1083.0);
      Scrollable.ensureVisible(findContext(6));
      await tester.pump();
      expect(tester.getTopLeft(findKey(6)).dy, equals(300.0));

      await prepare(735.0);
      Scrollable.ensureVisible(findContext(4), alignment: 1.0);
      await tester.pump();
      expect(tester.getBottomRight(findKey(4)).dy, equals(500.0));

      await prepare(123.0);
      Scrollable.ensureVisible(findContext(0), alignment: 1.0);
      await tester.pump();
      expect(tester.getTopLeft(findKey(0)).dy, equals(100.0));

      await prepare(523.0);
      Scrollable.ensureVisible(findContext(3), duration: const Duration(seconds: 1));
      await tester.pump();
      await tester.pump(const Duration(milliseconds: 1020));
      expect(tester.getTopLeft(findKey(3)).dy, equals(100.0));
    });

    testWidgets('ListView ensureVisible Axis.horizontal', (WidgetTester tester) async {
      BuildContext findContext(int i) => tester.element(findKey(i));
      Future<void> prepare(double offset) async {
        tester.state<ScrollableState>(find.byType(Scrollable)).position.jumpTo(offset);
        await tester.pump();
      }

      await tester.pumpWidget(buildListView(Axis.horizontal));

      await prepare(23.0);
      Scrollable.ensureVisible(findContext(3));
      await tester.pump();
      expect(tester.getTopLeft(findKey(3)).dx, equals(100.0));

      await prepare(843.0);
      Scrollable.ensureVisible(findContext(6));
      await tester.pump();
      expect(tester.getTopLeft(findKey(6)).dx, equals(500.0));

      await prepare(415.0);
      Scrollable.ensureVisible(findContext(4), alignment: 1.0);
      await tester.pump();
      expect(tester.getBottomRight(findKey(4)).dx, equals(700.0));

      await prepare(46.0);
      Scrollable.ensureVisible(findContext(0), alignment: 1.0);
      await tester.pump();
      expect(tester.getTopLeft(findKey(0)).dx, equals(100.0));

      await prepare(211.0);
      Scrollable.ensureVisible(findContext(3), duration: const Duration(seconds: 1));
      await tester.pump();
      await tester.pump(const Duration(milliseconds: 1020));
      expect(tester.getTopLeft(findKey(3)).dx, equals(100.0));
    });

    testWidgets('ListView ensureVisible Axis.vertical reverse', (WidgetTester tester) async {
      BuildContext findContext(int i) => tester.element(findKey(i));
      Future<void> prepare(double offset) async {
        tester.state<ScrollableState>(find.byType(Scrollable)).position.jumpTo(offset);
        await tester.pump();
      }

      await tester.pumpWidget(buildListView(Axis.vertical, reverse: true));

      await prepare(211.0);
      Scrollable.ensureVisible(findContext(3));
      await tester.pump();
      expect(tester.getBottomRight(findKey(3)).dy, equals(500.0));

      await prepare(23.0);
      Scrollable.ensureVisible(findContext(0));
      await tester.pump();
      expect(tester.getBottomRight(findKey(0)).dy, equals(500.0));

      await prepare(230.0);
      Scrollable.ensureVisible(findContext(2), alignment: 1.0);
      await tester.pump();
      expect(tester.getTopLeft(findKey(2)).dy, equals(100.0));

      await prepare(1083.0);
      Scrollable.ensureVisible(findContext(6), alignment: 1.0);
      await tester.pump();
      expect(tester.getBottomRight(findKey(6)).dy, equals(300.0));

      await prepare(345.0);
      Scrollable.ensureVisible(findContext(3), duration: const Duration(seconds: 1));
      await tester.pump();
      await tester.pump(const Duration(milliseconds: 1020));
      expect(tester.getBottomRight(findKey(3)).dy, equals(500.0));

      // Regression test for https://github.com/flutter/flutter/issues/128749
      // Reset to zero position.
      await prepare(0.0);
      // 2 is not currently visible as the ListView is contained
      // within a centered SizedBox.
      expect(tester.getBottomLeft(findKey(2)).dy, equals(100.0));
      expect(tester.getBottomLeft(findKey(0)).dy, equals(500.0));
      Scrollable.ensureVisible(
        findContext(0),
        alignmentPolicy: ScrollPositionAlignmentPolicy.keepVisibleAtStart,
      );
      await tester.pump();
      Scrollable.ensureVisible(
        findContext(1),
        alignmentPolicy: ScrollPositionAlignmentPolicy.keepVisibleAtStart,
      );
      await tester.pump();
      // 0 and 1 are already visible beyond the top edge, so no change.
      expect(tester.getBottomLeft(findKey(2)).dy, equals(100.0));
      expect(tester.getBottomLeft(findKey(0)).dy, equals(500.0));
      Scrollable.ensureVisible(
        findContext(2),
        alignmentPolicy: ScrollPositionAlignmentPolicy.keepVisibleAtStart,
      );
      await tester.pump();
      // Since it is reversed, 2 should have come into view at the top
      // edge of the scrollable, matching the alignment expectation.
      expect(tester.getBottomLeft(findKey(2)).dy, equals(300.0));
      expect(tester.getBottomLeft(findKey(0)).dy, equals(700.0));

      // Bring 0 back into view at the trailing edge, checking the other
      // alignment.
      Scrollable.ensureVisible(
        findContext(0),
        alignmentPolicy: ScrollPositionAlignmentPolicy.keepVisibleAtEnd,
      );
      await tester.pump();
      expect(tester.getBottomLeft(findKey(2)).dy, equals(100.0));
      expect(tester.getBottomLeft(findKey(0)).dy, equals(500.0));
    });

    testWidgets('ListView ensureVisible Axis.horizontal reverse', (WidgetTester tester) async {
      BuildContext findContext(int i) => tester.element(findKey(i));
      Future<void> prepare(double offset) async {
        tester.state<ScrollableState>(find.byType(Scrollable)).position.jumpTo(offset);
        await tester.pump();
      }

      await tester.pumpWidget(buildListView(Axis.horizontal, reverse: true));

      await prepare(211.0);
      Scrollable.ensureVisible(findContext(3));
      await tester.pump();
      expect(tester.getBottomRight(findKey(3)).dx, equals(700.0));

      await prepare(23.0);
      Scrollable.ensureVisible(findContext(0));
      await tester.pump();
      expect(tester.getBottomRight(findKey(0)).dx, equals(700.0));

      await prepare(230.0);
      Scrollable.ensureVisible(findContext(2), alignment: 1.0);
      await tester.pump();
      expect(tester.getTopLeft(findKey(2)).dx, equals(100.0));

      await prepare(1083.0);
      Scrollable.ensureVisible(findContext(6), alignment: 1.0);
      await tester.pump();
      expect(tester.getBottomRight(findKey(6)).dx, equals(300.0));

      await prepare(345.0);
      Scrollable.ensureVisible(findContext(3), duration: const Duration(seconds: 1));
      await tester.pump();
      await tester.pump(const Duration(milliseconds: 1020));
      expect(tester.getBottomRight(findKey(3)).dx, equals(700.0));

      // Regression test for https://github.com/flutter/flutter/issues/128749
      // Reset to zero position.
      await prepare(0.0);
      // 3 is not currently visible as the ListView is contained
      // within a centered SizedBox.
      expect(tester.getBottomLeft(findKey(3)).dx, equals(-100.0));
      expect(tester.getBottomLeft(findKey(0)).dx, equals(500.0));
      Scrollable.ensureVisible(
        findContext(0),
        alignmentPolicy: ScrollPositionAlignmentPolicy.keepVisibleAtStart,
      );
      await tester.pump();
      Scrollable.ensureVisible(
        findContext(1),
        alignmentPolicy: ScrollPositionAlignmentPolicy.keepVisibleAtStart,
      );
      await tester.pump();
      Scrollable.ensureVisible(
        findContext(2),
        alignmentPolicy: ScrollPositionAlignmentPolicy.keepVisibleAtStart,
      );
      await tester.pump();
      // 0, 1 and 2 are already visible beyond the left edge, so no change.
      expect(tester.getBottomLeft(findKey(3)).dx, equals(-100.0));
      expect(tester.getBottomLeft(findKey(0)).dx, equals(500.0));
      Scrollable.ensureVisible(
        findContext(3),
        alignmentPolicy: ScrollPositionAlignmentPolicy.keepVisibleAtStart,
      );
      await tester.pump();
      // Since it is reversed, 3 should have come into view at the leading
      // edge of the scrollable, matching the alignment expectation.
      expect(tester.getBottomLeft(findKey(3)).dx, equals(100.0));
      expect(tester.getBottomLeft(findKey(0)).dx, equals(700.0));

      // Bring 0 back into view at the trailing edge, checking the other
      // alignment.
      Scrollable.ensureVisible(
        findContext(0),
        alignmentPolicy: ScrollPositionAlignmentPolicy.keepVisibleAtEnd,
      );
      await tester.pump();
      expect(tester.getBottomLeft(findKey(3)).dx, equals(-100.0));
      expect(tester.getBottomLeft(findKey(0)).dx, equals(500.0));
    });

    testWidgets('ListView ensureVisible negative child', (WidgetTester tester) async {
      BuildContext findContext(int i) => tester.element(findKey(i));
      Future<void> prepare(double offset) async {
        tester.state<ScrollableState>(find.byType(Scrollable)).position.jumpTo(offset);
        await tester.pump();
      }

      double getOffset() {
        return tester.state<ScrollableState>(find.byType(Scrollable)).position.pixels;
      }

      Widget buildSliver(int i) {
        return SliverToBoxAdapter(
          key: ValueKey<int>(i),
          child: const SizedBox(width: 200.0, height: 200.0),
        );
      }

      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: Center(
            child: SizedBox(
              width: 600.0,
              height: 400.0,
              child: Scrollable(
                viewportBuilder: (BuildContext context, ViewportOffset offset) {
                  return Viewport(
                    offset: offset,
                    center: const ValueKey<int>(4),
                    slivers: <Widget>[
                      buildSliver(0),
                      buildSliver(1),
                      buildSliver(2),
                      buildSliver(3),
                      buildSliver(4),
                      buildSliver(5),
                      buildSliver(6),
                    ],
                  );
                },
              ),
            ),
          ),
        ),
      );

      await prepare(-125.0);
      Scrollable.ensureVisible(findContext(3));
      await tester.pump();
      expect(getOffset(), equals(-200.0));

      await prepare(-225.0);
      Scrollable.ensureVisible(findContext(2));
      await tester.pump();
      expect(getOffset(), equals(-400.0));
    });

    testWidgets('ListView ensureVisible rotated child', (WidgetTester tester) async {
      BuildContext findContext(int i) => tester.element(findKey(i));
      Future<void> prepare(double offset) async {
        tester.state<ScrollableState>(find.byType(Scrollable)).position.jumpTo(offset);
        await tester.pump();
      }

      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: Center(
            child: SizedBox(
              width: 600.0,
              height: 400.0,
              child: ListView(
                children: <Widget>[
                  const SizedBox(height: 200.0),
                  const SizedBox(height: 200.0),
                  const SizedBox(height: 200.0),
                  SizedBox(
                    height: 200.0,
                    child: Center(
                      child: Transform(
                        transform: Matrix4.rotationZ(math.pi),
                        child: Container(
                          key: const ValueKey<int>(0),
                          width: 100.0,
                          height: 100.0,
                          color: const Color(0xFFFFFFFF),
                        ),
                      ),
                    ),
                  ),
                  const SizedBox(height: 200.0),
                  const SizedBox(height: 200.0),
                  const SizedBox(height: 200.0),
                ],
              ),
            ),
          ),
        ),
      );

      await prepare(321.0);
      Scrollable.ensureVisible(findContext(0));
      await tester.pump();
      expect(tester.getBottomRight(findKey(0)).dy, moreOrLessEquals(100.0, epsilon: 0.1));

      Scrollable.ensureVisible(findContext(0), alignment: 1.0);
      await tester.pump();
      expect(tester.getTopLeft(findKey(0)).dy, moreOrLessEquals(500.0, epsilon: 0.1));
    });
  });

  group('ListView shrinkWrap', () {
    testWidgets('ListView ensureVisible Axis.vertical', (WidgetTester tester) async {
      BuildContext findContext(int i) => tester.element(findKey(i));
      Future<void> prepare(double offset) async {
        tester.state<ScrollableState>(find.byType(Scrollable)).position.jumpTo(offset);
        await tester.pump();
      }

      await tester.pumpWidget(buildListView(Axis.vertical, shrinkWrap: true));

      await prepare(480.0);
      Scrollable.ensureVisible(findContext(3));
      await tester.pump();
      expect(tester.getTopLeft(findKey(3)).dy, equals(100.0));

      await prepare(1083.0);
      Scrollable.ensureVisible(findContext(6));
      await tester.pump();
      expect(tester.getTopLeft(findKey(6)).dy, equals(300.0));

      await prepare(735.0);
      Scrollable.ensureVisible(findContext(4), alignment: 1.0);
      await tester.pump();
      expect(tester.getBottomRight(findKey(4)).dy, equals(500.0));

      await prepare(123.0);
      Scrollable.ensureVisible(findContext(0), alignment: 1.0);
      await tester.pump();
      expect(tester.getTopLeft(findKey(0)).dy, equals(100.0));

      await prepare(523.0);
      Scrollable.ensureVisible(findContext(3), duration: const Duration(seconds: 1));
      await tester.pump();
      await tester.pump(const Duration(milliseconds: 1020));
      expect(tester.getTopLeft(findKey(3)).dy, equals(100.0));
    });

    testWidgets('ListView ensureVisible Axis.horizontal', (WidgetTester tester) async {
      BuildContext findContext(int i) => tester.element(findKey(i));
      Future<void> prepare(double offset) async {
        tester.state<ScrollableState>(find.byType(Scrollable)).position.jumpTo(offset);
        await tester.pump();
      }

      await tester.pumpWidget(buildListView(Axis.horizontal, shrinkWrap: true));

      await prepare(23.0);
      Scrollable.ensureVisible(findContext(3));
      await tester.pump();
      expect(tester.getTopLeft(findKey(3)).dx, equals(100.0));

      await prepare(843.0);
      Scrollable.ensureVisible(findContext(6));
      await tester.pump();
      expect(tester.getTopLeft(findKey(6)).dx, equals(500.0));

      await prepare(415.0);
      Scrollable.ensureVisible(findContext(4), alignment: 1.0);
      await tester.pump();
      expect(tester.getBottomRight(findKey(4)).dx, equals(700.0));

      await prepare(46.0);
      Scrollable.ensureVisible(findContext(0), alignment: 1.0);
      await tester.pump();
      expect(tester.getTopLeft(findKey(0)).dx, equals(100.0));

      await prepare(211.0);
      Scrollable.ensureVisible(findContext(3), duration: const Duration(seconds: 1));
      await tester.pump();
      await tester.pump(const Duration(milliseconds: 1020));
      expect(tester.getTopLeft(findKey(3)).dx, equals(100.0));
    });

    testWidgets('ListView ensureVisible Axis.vertical reverse', (WidgetTester tester) async {
      BuildContext findContext(int i) => tester.element(findKey(i));
      Future<void> prepare(double offset) async {
        tester.state<ScrollableState>(find.byType(Scrollable)).position.jumpTo(offset);
        await tester.pump();
      }

      await tester.pumpWidget(buildListView(Axis.vertical, reverse: true, shrinkWrap: true));

      await prepare(211.0);
      Scrollable.ensureVisible(findContext(3));
      await tester.pump();
      expect(tester.getBottomRight(findKey(3)).dy, equals(500.0));

      await prepare(23.0);
      Scrollable.ensureVisible(findContext(0));
      await tester.pump();
      expect(tester.getBottomRight(findKey(0)).dy, equals(500.0));

      await prepare(230.0);
      Scrollable.ensureVisible(findContext(2), alignment: 1.0);
      await tester.pump();
      expect(tester.getTopLeft(findKey(2)).dy, equals(100.0));

      await prepare(1083.0);
      Scrollable.ensureVisible(findContext(6), alignment: 1.0);
      await tester.pump();
      expect(tester.getBottomRight(findKey(6)).dy, equals(300.0));

      await prepare(345.0);
      Scrollable.ensureVisible(findContext(3), duration: const Duration(seconds: 1));
      await tester.pump();
      await tester.pump(const Duration(milliseconds: 1020));
      expect(tester.getBottomRight(findKey(3)).dy, equals(500.0));

      // Regression test for https://github.com/flutter/flutter/issues/128749
      // Reset to zero position.
      await prepare(0.0);
      // 2 is not currently visible as the ListView is contained
      // within a centered SizedBox.
      expect(tester.getBottomLeft(findKey(2)).dy, equals(100.0));
      expect(tester.getBottomLeft(findKey(0)).dy, equals(500.0));
      Scrollable.ensureVisible(
        findContext(0),
        alignmentPolicy: ScrollPositionAlignmentPolicy.keepVisibleAtStart,
      );
      await tester.pump();
      Scrollable.ensureVisible(
        findContext(1),
        alignmentPolicy: ScrollPositionAlignmentPolicy.keepVisibleAtStart,
      );
      await tester.pump();
      // 0 and 1 are already visible beyond the top edge, so no change.
      expect(tester.getBottomLeft(findKey(2)).dy, equals(100.0));
      expect(tester.getBottomLeft(findKey(0)).dy, equals(500.0));
      Scrollable.ensureVisible(
        findContext(2),
        alignmentPolicy: ScrollPositionAlignmentPolicy.keepVisibleAtStart,
      );
      await tester.pump();
      // Since it is reversed, 2 should have come into view at the top
      // edge of the scrollable, matching the alignment expectation.
      expect(tester.getBottomLeft(findKey(2)).dy, equals(300.0));
      expect(tester.getBottomLeft(findKey(0)).dy, equals(700.0));

      // Bring 0 back into view at the trailing edge, checking the other
      // alignment.
      Scrollable.ensureVisible(
        findContext(0),
        alignmentPolicy: ScrollPositionAlignmentPolicy.keepVisibleAtEnd,
      );
      await tester.pump();
      expect(tester.getBottomLeft(findKey(2)).dy, equals(100.0));
      expect(tester.getBottomLeft(findKey(0)).dy, equals(500.0));
    });

    testWidgets('ListView ensureVisible Axis.horizontal reverse', (WidgetTester tester) async {
      BuildContext findContext(int i) => tester.element(findKey(i));
      Future<void> prepare(double offset) async {
        tester.state<ScrollableState>(find.byType(Scrollable)).position.jumpTo(offset);
        await tester.pump();
      }

      await tester.pumpWidget(buildListView(Axis.horizontal, reverse: true, shrinkWrap: true));

      await prepare(211.0);
      Scrollable.ensureVisible(findContext(3));
      await tester.pump();
      expect(tester.getBottomRight(findKey(3)).dx, equals(700.0));

      await prepare(23.0);
      Scrollable.ensureVisible(findContext(0));
      await tester.pump();
      expect(tester.getBottomRight(findKey(0)).dx, equals(700.0));

      await prepare(230.0);
      Scrollable.ensureVisible(findContext(2), alignment: 1.0);
      await tester.pump();
      expect(tester.getTopLeft(findKey(2)).dx, equals(100.0));

      await prepare(1083.0);
      Scrollable.ensureVisible(findContext(6), alignment: 1.0);
      await tester.pump();
      expect(tester.getBottomRight(findKey(6)).dx, equals(300.0));

      await prepare(345.0);
      Scrollable.ensureVisible(findContext(3), duration: const Duration(seconds: 1));
      await tester.pump();
      await tester.pump(const Duration(milliseconds: 1020));
      expect(tester.getBottomRight(findKey(3)).dx, equals(700.0));

      // Regression test for https://github.com/flutter/flutter/issues/128749
      // Reset to zero position.
      await prepare(0.0);
      // 3 is not currently visible as the ListView is contained
      // within a centered SizedBox.
      expect(tester.getBottomLeft(findKey(3)).dx, equals(-100.0));
      expect(tester.getBottomLeft(findKey(0)).dx, equals(500.0));
      Scrollable.ensureVisible(
        findContext(0),
        alignmentPolicy: ScrollPositionAlignmentPolicy.keepVisibleAtStart,
      );
      await tester.pump();
      Scrollable.ensureVisible(
        findContext(1),
        alignmentPolicy: ScrollPositionAlignmentPolicy.keepVisibleAtStart,
      );
      await tester.pump();
      Scrollable.ensureVisible(
        findContext(2),
        alignmentPolicy: ScrollPositionAlignmentPolicy.keepVisibleAtStart,
      );
      await tester.pump();
      // 0, 1 and 2 are already visible beyond the left edge, so no change.
      expect(tester.getBottomLeft(findKey(3)).dx, equals(-100.0));
      expect(tester.getBottomLeft(findKey(0)).dx, equals(500.0));
      Scrollable.ensureVisible(
        findContext(3),
        alignmentPolicy: ScrollPositionAlignmentPolicy.keepVisibleAtStart,
      );
      await tester.pump();
      // Since it is reversed, 3 should have come into view at the leading
      // edge of the scrollable, matching the alignment expectation.
      expect(tester.getBottomLeft(findKey(3)).dx, equals(100.0));
      expect(tester.getBottomLeft(findKey(0)).dx, equals(700.0));

      // Bring 0 back into view at the trailing edge, checking the other
      // alignment.
      Scrollable.ensureVisible(
        findContext(0),
        alignmentPolicy: ScrollPositionAlignmentPolicy.keepVisibleAtEnd,
      );
      await tester.pump();
      expect(tester.getBottomLeft(findKey(3)).dx, equals(-100.0));
      expect(tester.getBottomLeft(findKey(0)).dx, equals(500.0));
    });
  });

  group('Scrollable with center', () {
    testWidgets('ensureVisible', (WidgetTester tester) async {
      BuildContext findContext(int i) => tester.element(findKey(i));
      Future<void> prepare(double offset) async {
        tester.state<ScrollableState>(find.byType(Scrollable)).position.jumpTo(offset);
        await tester.pump();
      }

      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: Center(
            child: SizedBox(
              width: 600.0,
              height: 400.0,
              child: Scrollable(
                viewportBuilder: (BuildContext context, ViewportOffset offset) {
                  return Viewport(
                    offset: offset,
                    center: const ValueKey<String>('center'),
                    slivers: const <Widget>[
                      SliverToBoxAdapter(
                        child: SizedBox(key: ValueKey<int>(-6), width: 200.0, height: 200.0),
                      ),
                      SliverToBoxAdapter(
                        child: SizedBox(key: ValueKey<int>(-5), width: 200.0, height: 200.0),
                      ),
                      SliverToBoxAdapter(
                        child: SizedBox(key: ValueKey<int>(-4), width: 200.0, height: 200.0),
                      ),
                      SliverToBoxAdapter(
                        child: SizedBox(key: ValueKey<int>(-3), width: 200.0, height: 200.0),
                      ),
                      SliverToBoxAdapter(
                        child: SizedBox(key: ValueKey<int>(-2), width: 200.0, height: 200.0),
                      ),
                      SliverToBoxAdapter(
                        child: SizedBox(key: ValueKey<int>(-1), width: 200.0, height: 200.0),
                      ),
                      SliverToBoxAdapter(
                        key: ValueKey<String>('center'),
                        child: SizedBox(key: ValueKey<int>(0), width: 200.0, height: 200.0),
                      ),
                      SliverToBoxAdapter(
                        child: SizedBox(key: ValueKey<int>(1), width: 200.0, height: 200.0),
                      ),
                      SliverToBoxAdapter(
                        child: SizedBox(key: ValueKey<int>(2), width: 200.0, height: 200.0),
                      ),
                      SliverToBoxAdapter(
                        child: SizedBox(key: ValueKey<int>(3), width: 200.0, height: 200.0),
                      ),
                      SliverToBoxAdapter(
                        child: SizedBox(key: ValueKey<int>(4), width: 200.0, height: 200.0),
                      ),
                      SliverToBoxAdapter(
                        child: SizedBox(key: ValueKey<int>(5), width: 200.0, height: 200.0),
                      ),
                      SliverToBoxAdapter(
                        child: SizedBox(key: ValueKey<int>(6), width: 200.0, height: 200.0),
                      ),
                    ],
                  );
                },
              ),
            ),
          ),
        ),
      );

      await prepare(480.0);
      Scrollable.ensureVisible(findContext(3));
      await tester.pump();
      expect(tester.getTopLeft(findKey(3)).dy, equals(100.0));

      await prepare(1083.0);
      Scrollable.ensureVisible(findContext(6));
      await tester.pump();
      expect(tester.getTopLeft(findKey(6)).dy, equals(300.0));

      await prepare(735.0);
      Scrollable.ensureVisible(findContext(4), alignment: 1.0);
      await tester.pump();
      expect(tester.getBottomRight(findKey(4)).dy, equals(500.0));

      await prepare(123.0);
      Scrollable.ensureVisible(findContext(0), alignment: 1.0);
      await tester.pump();
      expect(tester.getBottomRight(findKey(0)).dy, equals(500.0));

      await prepare(523.0);
      Scrollable.ensureVisible(findContext(3), duration: const Duration(seconds: 1));
      await tester.pump();
      await tester.pump(const Duration(milliseconds: 1020));
      expect(tester.getTopLeft(findKey(3)).dy, equals(100.0));

      await prepare(-480.0);
      Scrollable.ensureVisible(findContext(-3));
      await tester.pump();
      expect(tester.getTopLeft(findKey(-3)).dy, equals(100.0));

      await prepare(-1083.0);
      Scrollable.ensureVisible(findContext(-6));
      await tester.pump();
      expect(tester.getTopLeft(findKey(-6)).dy, equals(100.0));

      await prepare(-735.0);
      Scrollable.ensureVisible(findContext(-4), alignment: 1.0);
      await tester.pump();
      expect(tester.getBottomRight(findKey(-4)).dy, equals(500.0));

      await prepare(-523.0);
      Scrollable.ensureVisible(findContext(-3), duration: const Duration(seconds: 1));
      await tester.pump();
      await tester.pump(const Duration(milliseconds: 1020));
      expect(tester.getTopLeft(findKey(-3)).dy, equals(100.0));
    });
  });

  group('TwoDimensionalViewport ensureVisible', () {
    Finder findKey(ChildVicinity vicinity) {
      return find.byKey(ValueKey<ChildVicinity>(vicinity));
    }

    BuildContext findContext(WidgetTester tester, ChildVicinity vicinity) {
      return tester.element(findKey(vicinity));
    }

    testWidgets('Axis.vertical', (WidgetTester tester) async {
      await tester.pumpWidget(simpleBuilderTest(useCacheExtent: true));

      Scrollable.ensureVisible(findContext(tester, const ChildVicinity(xIndex: 0, yIndex: 0)));
      await tester.pump();
      expect(tester.getTopLeft(findKey(const ChildVicinity(xIndex: 0, yIndex: 0))).dy, equals(0.0));
      // (0, 3) is in the cache extent, and will be brought into view next
      expect(
        tester.getTopLeft(findKey(const ChildVicinity(xIndex: 0, yIndex: 3))).dy,
        equals(600.0),
      );
      Scrollable.ensureVisible(findContext(tester, const ChildVicinity(xIndex: 0, yIndex: 3)));
      await tester.pump();
      // Now in view at top edge of viewport
      expect(tester.getTopLeft(findKey(const ChildVicinity(xIndex: 0, yIndex: 3))).dy, equals(0.0));

      // If already visible, no change
      Scrollable.ensureVisible(findContext(tester, const ChildVicinity(xIndex: 0, yIndex: 3)));
      await tester.pump();
      expect(tester.getTopLeft(findKey(const ChildVicinity(xIndex: 0, yIndex: 3))).dy, equals(0.0));
    });

    testWidgets('Axis.horizontal', (WidgetTester tester) async {
      await tester.pumpWidget(simpleBuilderTest(useCacheExtent: true));

      Scrollable.ensureVisible(findContext(tester, const ChildVicinity(xIndex: 1, yIndex: 0)));
      await tester.pump();
      expect(tester.getTopLeft(findKey(const ChildVicinity(xIndex: 1, yIndex: 0))).dx, equals(0.0));
      // (5, 0) is now in the cache extent, and will be brought into view next
      expect(
        tester.getTopLeft(findKey(const ChildVicinity(xIndex: 5, yIndex: 0))).dx,
        equals(800.0),
      );
      Scrollable.ensureVisible(
        findContext(tester, const ChildVicinity(xIndex: 5, yIndex: 0)),
        alignmentPolicy: ScrollPositionAlignmentPolicy.keepVisibleAtEnd,
      );
      await tester.pump();
      // Now in view at trailing edge of viewport
      expect(
        tester.getTopLeft(findKey(const ChildVicinity(xIndex: 5, yIndex: 0))).dx,
        equals(600.0),
      );

      // If already in position, no change
      Scrollable.ensureVisible(
        findContext(tester, const ChildVicinity(xIndex: 5, yIndex: 0)),
        alignmentPolicy: ScrollPositionAlignmentPolicy.keepVisibleAtEnd,
      );
      await tester.pump();
      expect(
        tester.getTopLeft(findKey(const ChildVicinity(xIndex: 5, yIndex: 0))).dx,
        equals(600.0),
      );
    });

    testWidgets('both axes', (WidgetTester tester) async {
      await tester.pumpWidget(simpleBuilderTest(useCacheExtent: true));

      Scrollable.ensureVisible(findContext(tester, const ChildVicinity(xIndex: 1, yIndex: 1)));
      await tester.pump();
      expect(
        tester.getRect(findKey(const ChildVicinity(xIndex: 1, yIndex: 1))),
        const Rect.fromLTRB(0.0, 0.0, 200.0, 200.0),
      );
      // (5, 4) is in the cache extent, and will be brought into view next
      expect(
        tester.getRect(findKey(const ChildVicinity(xIndex: 5, yIndex: 4))),
        const Rect.fromLTRB(800.0, 600.0, 1000.0, 800.0),
      );
      Scrollable.ensureVisible(
        findContext(tester, const ChildVicinity(xIndex: 5, yIndex: 4)),
        alignment: 1.0, // Same as ScrollAlignmentPolicy.keepVisibleAtEnd
      );
      await tester.pump();
      // Now in view at bottom trailing corner of viewport
      expect(
        tester.getRect(findKey(const ChildVicinity(xIndex: 5, yIndex: 4))),
        const Rect.fromLTRB(600.0, 400.0, 800.0, 600.0),
      );

      // If already visible, no change
      Scrollable.ensureVisible(
        findContext(tester, const ChildVicinity(xIndex: 5, yIndex: 4)),
        alignment: 1.0,
      );
      await tester.pump();
      expect(
        tester.getRect(findKey(const ChildVicinity(xIndex: 5, yIndex: 4))),
        const Rect.fromLTRB(600.0, 400.0, 800.0, 600.0),
      );
    });

    testWidgets('Axis.vertical reverse', (WidgetTester tester) async {
      await tester.pumpWidget(
        simpleBuilderTest(
          verticalDetails: const ScrollableDetails.vertical(reverse: true),
          useCacheExtent: true,
        ),
      );

      expect(
        tester.getTopLeft(findKey(const ChildVicinity(xIndex: 0, yIndex: 0))).dy,
        equals(400.0),
      );
      Scrollable.ensureVisible(findContext(tester, const ChildVicinity(xIndex: 0, yIndex: 0)));
      await tester.pump();
      // Already visible so no change.
      expect(
        tester.getTopLeft(findKey(const ChildVicinity(xIndex: 0, yIndex: 0))).dy,
        equals(400.0),
      );
      // (0, 3) is in the cache extent, and will be brought into view next
      expect(
        tester.getTopLeft(findKey(const ChildVicinity(xIndex: 0, yIndex: 3))).dy,
        equals(-200.0),
      );
      Scrollable.ensureVisible(findContext(tester, const ChildVicinity(xIndex: 0, yIndex: 3)));
      await tester.pump();
      // Now in view at bottom edge of viewport since we are reversed
      expect(
        tester.getTopLeft(findKey(const ChildVicinity(xIndex: 0, yIndex: 3))).dy,
        equals(400.0),
      );

      // If already visible, no change
      Scrollable.ensureVisible(findContext(tester, const ChildVicinity(xIndex: 0, yIndex: 3)));
      await tester.pump();
      expect(
        tester.getTopLeft(findKey(const ChildVicinity(xIndex: 0, yIndex: 3))).dy,
        equals(400.0),
      );
    });

    testWidgets('Axis.horizontal reverse', (WidgetTester tester) async {
      await tester.pumpWidget(
        simpleBuilderTest(
          horizontalDetails: const ScrollableDetails.horizontal(reverse: true),
          useCacheExtent: true,
        ),
      );

      expect(
        tester.getTopLeft(findKey(const ChildVicinity(xIndex: 0, yIndex: 0))).dx,
        equals(600.0),
      );
      Scrollable.ensureVisible(findContext(tester, const ChildVicinity(xIndex: 0, yIndex: 0)));
      await tester.pump();
      // Already visible so no change.
      expect(
        tester.getTopLeft(findKey(const ChildVicinity(xIndex: 0, yIndex: 0))).dx,
        equals(600.0),
      );
      // (4, 0) is in the cache extent, and will be brought into view next
      expect(
        tester.getTopLeft(findKey(const ChildVicinity(xIndex: 4, yIndex: 0))).dx,
        equals(-200.0),
      );
      Scrollable.ensureVisible(findContext(tester, const ChildVicinity(xIndex: 4, yIndex: 0)));
      await tester.pump();
      expect(
        tester.getTopLeft(findKey(const ChildVicinity(xIndex: 4, yIndex: 0))).dx,
        equals(200.0),
      );

      // If already visible, no change
      Scrollable.ensureVisible(findContext(tester, const ChildVicinity(xIndex: 4, yIndex: 0)));
      await tester.pump();
      expect(
        tester.getTopLeft(findKey(const ChildVicinity(xIndex: 4, yIndex: 0))).dx,
        equals(200.0),
      );
    });

    testWidgets('both axes reverse', (WidgetTester tester) async {
      await tester.pumpWidget(
        simpleBuilderTest(
          verticalDetails: const ScrollableDetails.vertical(reverse: true),
          horizontalDetails: const ScrollableDetails.horizontal(reverse: true),
          useCacheExtent: true,
        ),
      );

      Scrollable.ensureVisible(findContext(tester, const ChildVicinity(xIndex: 1, yIndex: 1)));
      await tester.pump();
      expect(
        tester.getRect(findKey(const ChildVicinity(xIndex: 1, yIndex: 1))),
        const Rect.fromLTRB(600.0, 400.0, 800.0, 600.0),
      );
      // (5, 4) is in the cache extent, and will be brought into view next
      expect(
        tester.getRect(findKey(const ChildVicinity(xIndex: 5, yIndex: 4))),
        const Rect.fromLTRB(-200.0, -200.0, 0.0, 0.0),
      );
      Scrollable.ensureVisible(
        findContext(tester, const ChildVicinity(xIndex: 5, yIndex: 4)),
        alignment: 1.0, // Same as ScrollAlignmentPolicy.keepVisibleAtEnd
      );
      await tester.pump();
      // Now in view at trailing corner of viewport
      expect(
        tester.getRect(findKey(const ChildVicinity(xIndex: 5, yIndex: 4))),
        const Rect.fromLTRB(0.0, 0.0, 200.0, 200.0),
      );

      // If already visible, no change
      Scrollable.ensureVisible(
        findContext(tester, const ChildVicinity(xIndex: 5, yIndex: 4)),
        alignment: 1.0,
      );
      await tester.pump();
      expect(
        tester.getRect(findKey(const ChildVicinity(xIndex: 5, yIndex: 4))),
        const Rect.fromLTRB(0.0, 0.0, 200.0, 200.0),
      );
    });
  });
}
