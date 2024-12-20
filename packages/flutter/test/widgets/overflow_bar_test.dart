// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('OverflowBar documented defaults', (WidgetTester tester) async {
    const OverflowBar bar = OverflowBar();
    expect(bar.spacing, 0);
    expect(bar.alignment, null);
    expect(bar.overflowSpacing, 0);
    expect(bar.overflowDirection, VerticalDirection.down);
    expect(bar.textDirection, null);
    expect(bar.children, const <Widget>[]);
  });

  testWidgets('Empty OverflowBar', (WidgetTester tester) async {
    const Size size = Size(16, 24);

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Center(
          child: ConstrainedBox(
            constraints: BoxConstraints.tight(size),
            child: const OverflowBar(),
          ),
        ),
      ),
    );

    expect(tester.getSize(find.byType(OverflowBar)), size);

    await tester.pumpWidget(
      const Directionality(textDirection: TextDirection.ltr, child: Center(child: OverflowBar())),
    );

    expect(tester.getSize(find.byType(OverflowBar)), Size.zero);
  });

  testWidgets('OverflowBar horizontal layout', (WidgetTester tester) async {
    final Key child1Key = UniqueKey();
    final Key child2Key = UniqueKey();
    final Key child3Key = UniqueKey();

    Widget buildFrame({required double spacing, required TextDirection textDirection}) {
      return Directionality(
        textDirection: textDirection,
        child: Align(
          alignment: Alignment.topLeft,
          child: OverflowBar(
            spacing: spacing,
            children: <Widget>[
              SizedBox(width: 48, height: 48, key: child1Key),
              SizedBox(width: 64, height: 64, key: child2Key),
              SizedBox(width: 32, height: 32, key: child3Key),
            ],
          ),
        ),
      );
    }

    // Children are vertically centered, start at x=0
    await tester.pumpWidget(buildFrame(spacing: 0, textDirection: TextDirection.ltr));
    expect(tester.getRect(find.byKey(child1Key)), const Rect.fromLTRB(0, 8, 48, 56));
    expect(tester.getRect(find.byKey(child2Key)), const Rect.fromLTRB(48, 0, 112, 64));
    expect(tester.getRect(find.byKey(child3Key)), const Rect.fromLTRB(112, 16, 144, 48));

    // Children are vertically centered, start at x=0
    await tester.pumpWidget(buildFrame(spacing: 10, textDirection: TextDirection.ltr));
    expect(tester.getRect(find.byKey(child1Key)), const Rect.fromLTRB(0, 8, 48, 56));
    expect(
      tester.getRect(find.byKey(child2Key)),
      const Rect.fromLTRB(10.0 + 48, 0, 10.0 + 112, 64),
    );
    expect(
      tester.getRect(find.byKey(child3Key)),
      const Rect.fromLTRB(10.0 + 112 + 10.0, 16, 10.0 + 10.0 + 144, 48),
    );

    // Children appear in reverse order for RTL
    await tester.pumpWidget(buildFrame(spacing: 0, textDirection: TextDirection.rtl));
    expect(tester.getRect(find.byKey(child3Key)), const Rect.fromLTRB(0, 16, 32, 48));
    expect(tester.getRect(find.byKey(child2Key)), const Rect.fromLTRB(32, 0, 96, 64));
    expect(tester.getRect(find.byKey(child1Key)), const Rect.fromLTRB(96, 8, 144, 56));

    // Children appear in reverse order for RTL
    await tester.pumpWidget(buildFrame(spacing: 10, textDirection: TextDirection.rtl));
    expect(tester.getRect(find.byKey(child3Key)), const Rect.fromLTRB(0, 16, 32, 48));
    expect(tester.getRect(find.byKey(child2Key)), const Rect.fromLTRB(10.0 + 32, 0, 10.0 + 96, 64));
    expect(
      tester.getRect(find.byKey(child1Key)),
      const Rect.fromLTRB(10.0 + 96 + 10.0, 8, 10.0 + 10.0 + 144, 56),
    );
  });

  testWidgets('OverflowBar vertical layout', (WidgetTester tester) async {
    final Key child1Key = UniqueKey();
    final Key child2Key = UniqueKey();
    final Key child3Key = UniqueKey();

    Widget buildFrame({
      double overflowSpacing = 0,
      VerticalDirection overflowDirection = VerticalDirection.down,
      OverflowBarAlignment overflowAlignment = OverflowBarAlignment.start,
      TextDirection textDirection = TextDirection.ltr,
    }) {
      return Directionality(
        textDirection: textDirection,
        child: Align(
          alignment: Alignment.topLeft,
          child: ConstrainedBox(
            constraints: BoxConstraints.loose(const Size(100, double.infinity)),
            child: OverflowBar(
              overflowSpacing: overflowSpacing,
              overflowAlignment: overflowAlignment,
              overflowDirection: overflowDirection,
              children: <Widget>[
                SizedBox(width: 48, height: 48, key: child1Key),
                SizedBox(width: 64, height: 64, key: child2Key),
                SizedBox(width: 32, height: 32, key: child3Key),
              ],
            ),
          ),
        ),
      );
    }

    // Children are left aligned
    await tester.pumpWidget(buildFrame());
    expect(tester.getRect(find.byKey(child1Key)), const Rect.fromLTRB(0, 0, 48, 48));
    expect(tester.getRect(find.byKey(child2Key)), const Rect.fromLTRB(0, 48, 64, 112));
    expect(tester.getRect(find.byKey(child3Key)), const Rect.fromLTRB(0, 112, 32, 144));

    // Children are left aligned
    await tester.pumpWidget(
      buildFrame(overflowAlignment: OverflowBarAlignment.end, textDirection: TextDirection.rtl),
    );
    expect(tester.getRect(find.byKey(child1Key)), const Rect.fromLTRB(0, 0, 48, 48));
    expect(tester.getRect(find.byKey(child2Key)), const Rect.fromLTRB(0, 48, 64, 112));
    expect(tester.getRect(find.byKey(child3Key)), const Rect.fromLTRB(0, 112, 32, 144));

    // Spaced children are left aligned
    await tester.pumpWidget(buildFrame(overflowSpacing: 10));
    expect(tester.getRect(find.byKey(child1Key)), const Rect.fromLTRB(0, 0, 48, 48));
    expect(
      tester.getRect(find.byKey(child2Key)),
      const Rect.fromLTRB(0, 10.0 + 48, 64, 10.0 + 112),
    );
    expect(
      tester.getRect(find.byKey(child3Key)),
      const Rect.fromLTRB(0, 10.0 + 112 + 10.0, 32, 10.0 + 10.0 + 144),
    );

    // Left-aligned children appear in reverse order for VerticalDirection.up
    await tester.pumpWidget(buildFrame(overflowDirection: VerticalDirection.up));
    expect(tester.getRect(find.byKey(child3Key)), const Rect.fromLTRB(0, 0, 32, 32));
    expect(tester.getRect(find.byKey(child2Key)), const Rect.fromLTRB(0, 32, 64, 96));
    expect(tester.getRect(find.byKey(child1Key)), const Rect.fromLTRB(0, 96, 48, 144));

    // Left-aligned spaced children appear in reverse order for VerticalDirection.up
    await tester.pumpWidget(
      buildFrame(overflowSpacing: 10, overflowDirection: VerticalDirection.up),
    );
    expect(tester.getRect(find.byKey(child3Key)), const Rect.fromLTRB(0, 0, 32, 32));
    expect(tester.getRect(find.byKey(child2Key)), const Rect.fromLTRB(0, 10.0 + 32, 64, 10.0 + 96));
    expect(
      tester.getRect(find.byKey(child1Key)),
      const Rect.fromLTRB(0, 10.0 + 10.0 + 96, 48, 10.0 + 10.0 + 144),
    );

    // Children are right aligned
    await tester.pumpWidget(buildFrame(overflowAlignment: OverflowBarAlignment.end));
    expect(tester.getRect(find.byKey(child1Key)), const Rect.fromLTRB(100.0 - 48, 0, 100, 48));
    expect(tester.getRect(find.byKey(child2Key)), const Rect.fromLTRB(100.0 - 64, 48, 100, 112));
    expect(tester.getRect(find.byKey(child3Key)), const Rect.fromLTRB(100.0 - 32, 112, 100, 144));

    // Children are right aligned
    await tester.pumpWidget(buildFrame(textDirection: TextDirection.rtl));
    expect(tester.getRect(find.byKey(child1Key)), const Rect.fromLTRB(100.0 - 48, 0, 100, 48));
    expect(tester.getRect(find.byKey(child2Key)), const Rect.fromLTRB(100.0 - 64, 48, 100, 112));
    expect(tester.getRect(find.byKey(child3Key)), const Rect.fromLTRB(100.0 - 32, 112, 100, 144));

    // Children are centered
    await tester.pumpWidget(buildFrame(overflowAlignment: OverflowBarAlignment.center));
    expect(
      tester.getRect(find.byKey(child1Key)),
      const Rect.fromLTRB(100.0 / 2.0 - 48 / 2, 0, 100.0 / 2.0 + 48 / 2, 48),
    );
    expect(
      tester.getRect(find.byKey(child2Key)),
      const Rect.fromLTRB(100.0 / 2.0 - 64 / 2, 48, 100.0 / 2.0 + 64 / 2, 112),
    );
    expect(
      tester.getRect(find.byKey(child3Key)),
      const Rect.fromLTRB(100.0 / 2.0 - 32 / 2, 112, 100.0 / 2.0 + 32 / 2, 144),
    );
  });

  testWidgets('OverflowBar intrinsic width', (WidgetTester tester) async {
    Widget buildFrame({required double width}) {
      return Directionality(
        textDirection: TextDirection.ltr,
        child: Center(
          child: Container(
            width: width,
            alignment: Alignment.topLeft,
            child: const IntrinsicWidth(
              child: OverflowBar(
                spacing: 4,
                overflowSpacing: 8,
                children: <Widget>[
                  SizedBox(width: 48, height: 50),
                  SizedBox(width: 64, height: 25),
                  SizedBox(width: 32, height: 75),
                ],
              ),
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildFrame(width: 800));
    expect(tester.getSize(find.byType(OverflowBar)).width, 152); // 152 = 48 + 4 + 64 + 4 + 32

    await tester.pumpWidget(buildFrame(width: 150));
    expect(tester.getSize(find.byType(OverflowBar)).width, 150);
  });

  testWidgets('OverflowBar intrinsic height', (WidgetTester tester) async {
    Widget buildFrame({required double maxWidth}) {
      return Directionality(
        textDirection: TextDirection.ltr,
        child: Center(
          child: Container(
            width: maxWidth,
            alignment: Alignment.topLeft,
            child: const IntrinsicHeight(
              child: OverflowBar(
                spacing: 4,
                overflowSpacing: 8,
                children: <Widget>[
                  SizedBox(width: 48, height: 50),
                  SizedBox(width: 64, height: 25),
                  SizedBox(width: 32, height: 75),
                ],
              ),
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildFrame(maxWidth: 800));
    expect(tester.getSize(find.byType(OverflowBar)).height, 75); // 75 = max(50, 25, 75)

    await tester.pumpWidget(buildFrame(maxWidth: 150));
    expect(tester.getSize(find.byType(OverflowBar)).height, 166); // 166 = 50 + 8 + 25 + 8 + 75
  });

  testWidgets('OverflowBar is wider that its intrinsic width', (WidgetTester tester) async {
    final Key key0 = UniqueKey();
    final Key key1 = UniqueKey();
    final Key key2 = UniqueKey();

    Widget buildFrame(TextDirection textDirection) {
      return Directionality(
        textDirection: textDirection,
        child: SizedBox(
          width: 800,
          // intrinsic width = 50 + 10 + 60 + 10 + 70 = 200
          child: OverflowBar(
            spacing: 10,
            children: <Widget>[
              SizedBox(key: key0, width: 50, height: 50),
              SizedBox(key: key1, width: 60, height: 50),
              SizedBox(key: key2, width: 70, height: 50),
            ],
          ),
        ),
      );
    }

    await tester.pumpWidget(buildFrame(TextDirection.ltr));
    expect(tester.getSize(find.byType(OverflowBar)), const Size(800.0, 600.0));
    expect(tester.getTopLeft(find.byKey(key0)).dx, 0);
    expect(tester.getTopLeft(find.byKey(key1)).dx, 60);
    expect(tester.getTopLeft(find.byKey(key2)).dx, 130);

    await tester.pumpWidget(buildFrame(TextDirection.rtl));
    expect(tester.getSize(find.byType(OverflowBar)), const Size(800.0, 600.0));
    expect(tester.getTopLeft(find.byKey(key0)).dx, 750);
    expect(tester.getTopLeft(find.byKey(key1)).dx, 680);
    expect(tester.getTopLeft(find.byKey(key2)).dx, 600);
  });

  testWidgets('OverflowBar with alignment should match Row with mainAxisAlignment', (
    WidgetTester tester,
  ) async {
    final Key key0 = UniqueKey();
    final Key key1 = UniqueKey();
    final Key key2 = UniqueKey();

    // This list of children appears in a Row and an OverflowBar, so each
    // find.byKey() for key0, key1, key2 returns two widgets.
    final List<Widget> children = <Widget>[
      SizedBox(key: key0, width: 50, height: 50),
      SizedBox(key: key1, width: 70, height: 50),
      SizedBox(key: key2, width: 80, height: 50),
    ];

    const List<MainAxisAlignment> allAlignments = <MainAxisAlignment>[
      MainAxisAlignment.start,
      MainAxisAlignment.center,
      MainAxisAlignment.end,
      MainAxisAlignment.spaceBetween,
      MainAxisAlignment.spaceAround,
      MainAxisAlignment.spaceEvenly,
    ];

    const List<TextDirection> allTextDirections = <TextDirection>[
      TextDirection.ltr,
      TextDirection.rtl,
    ];

    Widget buildFrame(MainAxisAlignment alignment, TextDirection textDirection) {
      return Directionality(
        textDirection: textDirection,
        child: Column(
          children: <Widget>[
            OverflowBar(alignment: alignment, children: children),
            Row(mainAxisAlignment: alignment, children: children),
          ],
        ),
      );
    }

    // Each key from key0, key1, key2 maps to one child in the OverflowBar
    // and a matching child in the Row. We expect the children to be the
    // same size and for their left and right edges to align.
    void testLayout() {
      expect(tester.getSize(find.byType(OverflowBar)), const Size(800, 50));
      for (final Key key in <Key>[key0, key1, key2]) {
        final Finder matchingChildren = find.byKey(key);
        expect(matchingChildren.evaluate().length, 2);
        final Rect rect0 = tester.getRect(matchingChildren.first);
        final Rect rect1 = tester.getRect(matchingChildren.last);
        expect(rect0.size, rect1.size);
        expect(rect0.left, rect1.left);
        expect(rect0.right, rect1.right);
      }
    }

    for (final MainAxisAlignment alignment in allAlignments) {
      for (final TextDirection textDirection in allTextDirections) {
        await tester.pumpWidget(buildFrame(alignment, textDirection));
        testLayout();
      }
    }
  });
}
